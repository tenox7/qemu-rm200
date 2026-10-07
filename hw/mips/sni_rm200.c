/*
 * Siemens Nixdorf RM200C (PCI desktop, "ASIC PCI" / PCIMT chipset)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qemu/datadir.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "chardev/char.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/mips/mips.h"
#include "hw/intc/i8259.h"
#include "hw/dma/i8257.h"
#include "hw/isa/isa.h"
#include "hw/char/serial-isa.h"
#include "hw/char/parallel.h"
#include "hw/char/parallel-isa.h"
#include "hw/block/fdc.h"
#include "hw/rtc/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "hw/audio/pcspk.h"
#include "hw/input/i8042.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/qtest.h"
#include "target/mips/cpu.h"
#include "trace.h"

#define PROM_BASE       0x1fc00000
#define PROM_SIZE       (512 * KiB)
#define XBUS_BASE       0x1fd00000
#define NVRAM_BASE      0x1ff00000
#define NVRAM_SIZE      (32 * KiB)
#define ASIC_BASE       0x1fff0000
#define RM_ISA_MEM_BASE    0x10000000
#define RM_ISA_MEM_SIZE    0x04000000
#define RM_PCI_MEM_BASE    0x18000000
#define RM_PCI_MEM_SIZE    0x07c00000
#define RM_PCI_IO_BASE     0x14000000
#define RM_PCI_IO_SIZE     0x03c00000
#define CACHE_REPL_BASE 0x17c00000
#define CACHE_REPL_SIZE 0x00400000
#define INTACK_BASE     0x1a000000

/* ITPEND / IRQSEL / CSITPEND bits */
#define IT_INT2  0x01
#define IT_INTD  0x02
#define IT_INTC  0x04
#define IT_INTB  0x08
#define IT_INTA  0x10
#define IT_EISA  0x20
#define IT_SCSI  0x40
#define IT_ETH   0x80
#define IT_PCI   (IT_INTA | IT_INTB | IT_INTC | IT_INTD)

#define SCSI_IRQ_DELAY_NS   (200 * SCALE_US)

/* ASIC PCI registers, big-endian offsets */
#define ASIC_UCONF          0x04
#define ASIC_IOADTIMEOUT2   0x0c
#define ASIC_IOMEMCONF      0x14
#define ASIC_IOMMU          0x1c
#define ASIC_IOADTIMEOUT1   0x24
#define ASIC_DMAACCESS      0x2c
#define ASIC_DMAHIT         0x34
#define ASIC_ERRSTATUS      0x3c
#define ASIC_ERRADDR        0x44
#define ASIC_SYNDROME       0x4c
#define ASIC_ITPEND         0x54
#define ASIC_IRQSEL         0x5c
#define ASIC_TESTMEM        0x64
#define ASIC_ECCREG         0x6c
#define ASIC_CONFIG_ADDRESS 0x74
#define ASIC_ID             0x7c
#define ASIC_PIA_OE         0x84
#define ASIC_PIA_DATAOUT    0x8c
#define ASIC_PIA_DATAIN     0x94
#define ASIC_CACHECONF      0x9c
#define ASIC_INVSPACE       0xa4
#define ASIC_REGS_SIZE      0x1000

/* X-bus board registers, 64k apart */
enum {
    XB_CSMSR, XB_CSSWITCH, XB_CSITPEND, XB_AUTO_PO_EN, XB_CLR_TEMP,
    XB_AUTO_PO_DIS, XB_EXMSR, XB_UNUSED1, XB_CSWCSM, XB_UNUSED2, XB_CSLED,
    XB_CSMAPISA, XB_CSRSTBP, XB_CLRPOFF, XB_CSTIMER, XB_PWDN,
};

static const char *const xbus_names[16] = {
    "CSMSR", "CSSWITCH", "CSITPEND", "AUTO_PO_EN", "CLR_TEMP", "AUTO_PO_DIS",
    "EXMSR", "UNUSED1", "CSWCSM", "UNUSED2", "CSLED", "CSMAPISA", "CSRSTBP",
    "CLRPOFF", "CSTIMER", "PWDN",
};

#define TYPE_SNI_RM200_MACHINE MACHINE_TYPE_NAME("sni-rm200")
OBJECT_DECLARE_SIMPLE_TYPE(SniRm200MachineState, SNI_RM200_MACHINE)

struct SniRm200MachineState {
    MachineState parent_obj;
    char *nvram_file;
};

#define TYPE_SNI_PCIMT "sni-pcimt-pcihost"
OBJECT_DECLARE_SIMPLE_TYPE(SniPcimtState, SNI_PCIMT)

struct SniPcimtState {
    PCIHostState parent_obj;

    MemoryRegion asic_mr;
    MemoryRegion xbus_mr;
    MemoryRegion cfg_mr;
    MemoryRegion intack_mr;
    MemoryRegion pci_mem;
    MemoryRegion pci_io;
    MemoryRegion isa_mem_win;
    MemoryRegion pci_mem_win;
    MemoryRegion pci_io_win;
    MemoryRegion io_bg;
    MemoryRegion mem_bg;

    qemu_irq cpu_irq[5];
    QEMUTimer *scsi_timer;
    uint32_t asic[ASIC_REGS_SIZE / 4];
    uint8_t xbus[16];
    uint8_t pend;
    uint8_t csmsr;
    uint8_t csswitch;
};

/*
 * IRQSEL is not a plain mask (firmware writes 0x0005a33f, SINIX 0x0005b7bf
 * and still expects SCSI on IP3), so sources go straight to the CPU lines.
 */
static void pcimt_update_irq(SniPcimtState *s)
{
    uint8_t act = s->pend;

    qemu_set_irq(s->cpu_irq[0], !!(act & IT_INT2));
    qemu_set_irq(s->cpu_irq[1], !!(act & (IT_EISA | IT_SCSI)));
    qemu_set_irq(s->cpu_irq[3], !!(act & IT_PCI));
    qemu_set_irq(s->cpu_irq[4], !!(act & IT_ETH));
}

static void pcimt_scsi_irq(void *opaque)
{
    SniPcimtState *s = opaque;

    s->pend |= IT_SCSI;
    pcimt_update_irq(s);
}

/*
 * Real disks take milliseconds. SINIX enters init with a stale splhi(), so a
 * SCSI completion arriving within microseconds can slip between biowait()'s
 * B_DONE check and sleep(), losing the wakeup and hanging the boot.
 */
static void pcimt_set_pend(void *opaque, int bit, int level)
{
    SniPcimtState *s = opaque;

    if (1 << bit == IT_SCSI && level) {
        if (!(s->pend & IT_SCSI) && !timer_pending(s->scsi_timer)) {
            timer_mod(s->scsi_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SCSI_IRQ_DELAY_NS);
        }
        return;
    }
    if (1 << bit == IT_SCSI) {
        timer_del(s->scsi_timer);
    }
    if (level) {
        s->pend |= 1 << bit;
    } else {
        s->pend &= ~(1 << bit);
    }
    pcimt_update_irq(s);
}

static uint64_t asic_read(void *opaque, hwaddr addr, unsigned size)
{
    SniPcimtState *s = opaque;
    uint32_t val = s->asic[addr >> 2];

    if (addr == ASIC_ITPEND) {
        val = s->pend;
    }
    trace_sni_asic_read(addr, val);
    return val;
}

static void asic_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SniPcimtState *s = opaque;

    trace_sni_asic_write(addr, val);
    switch (addr) {
    case ASIC_ID:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    case ASIC_ITPEND:
        return;
    }
    s->asic[addr >> 2] = val;
    if (addr == ASIC_IRQSEL) {
        pcimt_update_irq(s);
    }
}

static const MemoryRegionOps asic_ops = {
    .read = asic_read,
    .write = asic_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static uint8_t xbus_get(SniPcimtState *s, int reg)
{
    switch (reg) {
    case XB_CSMSR:
        return s->csmsr;
    case XB_CSSWITCH:
        return s->csswitch;
    case XB_CSITPEND:
        return (s->pend & IT_EISA) | (~s->pend & ~IT_EISA & 0xff);
    }
    return s->xbus[reg];
}

static uint64_t xbus_read(void *opaque, hwaddr addr, unsigned size)
{
    SniPcimtState *s = opaque;
    int reg = addr >> 16;
    uint8_t v = xbus_get(s, reg);

    trace_sni_xbus_read(xbus_names[reg], v);
    return v * 0x01010101u;
}

static void xbus_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SniPcimtState *s = opaque;
    int reg = addr >> 16;

    trace_sni_xbus_write(xbus_names[reg], val);
    s->xbus[reg] = val;
    if (reg == XB_CSWCSM && (val & 0xff) == 0xfd) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
}

static const MemoryRegionOps xbus_ops = {
    .read = xbus_read,
    .write = xbus_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static uint64_t pcicfg_read(void *opaque, hwaddr addr, unsigned size)
{
    SniPcimtState *s = opaque;
    uint32_t a = (s->asic[ASIC_CONFIG_ADDRESS >> 2] & 0x00fffffc) | addr;
    uint32_t val = pci_data_read(PCI_HOST_BRIDGE(s)->bus, a, size);

    trace_sni_pcicfg_read(a, size, val);
    return val;
}

static void pcicfg_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    SniPcimtState *s = opaque;
    uint32_t a = (s->asic[ASIC_CONFIG_ADDRESS >> 2] & 0x00fffffc) | addr;

    trace_sni_pcicfg_write(a, size, val);
    pci_data_write(PCI_HOST_BRIDGE(s)->bus, a, val, size);
}

static const MemoryRegionOps pcicfg_ops = {
    .read = pcicfg_read,
    .write = pcicfg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static uint64_t intack_read(void *opaque, hwaddr addr, unsigned size)
{
    return pic_read_irq(isa_pic);
}

static void intack_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
}

static const MemoryRegionOps intack_ops = {
    .read = intack_read,
    .write = intack_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t bg_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "%s: read  0x%08" HWADDR_PRIx "/%d\n",
                  (const char *)opaque, addr, size);
    return ~0ull;
}

static void bg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "%s: write 0x%08" HWADDR_PRIx "/%d = 0x%" PRIx64
                  "\n", (const char *)opaque, addr, size, val);
}

static const MemoryRegionOps bg_ops = {
    .read = bg_read,
    .write = bg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/* PCI slot/pin -> pending bit, from Linux irq_tab_rm200 */
static int pcimt_map_irq(PCIDevice *d, int pin)
{
    static const uint8_t tab[8][4] = {
        { 0, 0, 0, 0 },     /* EISA bridge */
        { 6, 6, 6, 6 },     /* SCSI */
        { 7, 7, 7, 7 },     /* Ethernet */
        { 3, 3, 3, 3 },     /* VGA -> INTB */
        { 0, 0, 0, 0 },
        { 3, 2, 1, 4 },     /* slot 2 */
        { 2, 1, 4, 3 },     /* slot 3 */
        { 1, 4, 3, 2 },     /* slot 4 */
    };

    return tab[PCI_SLOT(d->devfn) & 7][pin & 3];
}

static void pcimt_reset(DeviceState *dev)
{
    SniPcimtState *s = SNI_PCIMT(dev);

    memset(s->asic, 0, sizeof(s->asic));
    memset(s->xbus, 0, sizeof(s->xbus));
    s->pend &= IT_EISA | IT_SCSI | IT_ETH | IT_PCI;
    pcimt_update_irq(s);
}

static void pcimt_realize(DeviceState *dev, Error **errp)
{
    SniPcimtState *s = SNI_PCIMT(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init(&s->pci_mem, OBJECT(s), "pci-mem", 4 * GiB);
    memory_region_init(&s->pci_io, OBJECT(s), "pci-io", RM_PCI_IO_SIZE);
    memory_region_init_io(&s->mem_bg, OBJECT(s), &bg_ops, (void *)"pcimem-bg",
                          "pci-mem-bg", 4 * GiB);
    memory_region_add_subregion_overlap(&s->pci_mem, 0, &s->mem_bg, -100);
    memory_region_init_io(&s->io_bg, OBJECT(s), &bg_ops, (void *)"pciio-bg",
                          "pci-io-bg", RM_PCI_IO_SIZE);
    memory_region_add_subregion_overlap(&s->pci_io, 0, &s->io_bg, -100);

    memory_region_init_alias(&s->isa_mem_win, OBJECT(s), "isa-mem-win",
                             &s->pci_mem, 0, RM_ISA_MEM_SIZE);
    memory_region_init_alias(&s->pci_mem_win, OBJECT(s), "pci-mem-win",
                             &s->pci_mem, RM_PCI_MEM_BASE, RM_PCI_MEM_SIZE);
    memory_region_init_alias(&s->pci_io_win, OBJECT(s), "pci-io-win",
                             &s->pci_io, 0, RM_PCI_IO_SIZE);
    sysbus_init_mmio(sbd, &s->isa_mem_win);
    sysbus_init_mmio(sbd, &s->pci_mem_win);
    sysbus_init_mmio(sbd, &s->pci_io_win);

    memory_region_init_io(&s->asic_mr, OBJECT(s), &asic_ops, s, "asic-pci",
                          ASIC_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->asic_mr);
    memory_region_init_io(&s->xbus_mr, OBJECT(s), &xbus_ops, s, "xbus-regs",
                          16 * 64 * KiB);
    sysbus_init_mmio(sbd, &s->xbus_mr);
    memory_region_init_io(&s->intack_mr, OBJECT(s), &intack_ops, s,
                          "pci-intack", 16 * MiB);
    sysbus_init_mmio(sbd, &s->intack_mr);

    memory_region_init_io(&s->cfg_mr, OBJECT(s), &pcicfg_ops, s, "pci-conf",
                          4);
    memory_region_add_subregion(&s->pci_io, 0xcfc, &s->cfg_mr);

    for (i = 0; i < ARRAY_SIZE(s->cpu_irq); i++) {
        sysbus_init_irq(sbd, &s->cpu_irq[i]);
    }
    qdev_init_gpio_in(dev, pcimt_set_pend, 8);
    s->scsi_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pcimt_scsi_irq, s);

    phb->bus = pci_register_root_bus(dev, "pci", pcimt_set_pend,
                                     pcimt_map_irq, s, &s->pci_mem,
                                     &s->pci_io, 0, 8, TYPE_PCI_BUS);
}

static const Property pcimt_props[] = {
    DEFINE_PROP_UINT8("csmsr", SniPcimtState, csmsr, 0x87),
    DEFINE_PROP_UINT8("csswitch", SniPcimtState, csswitch, 0x00),
};

static void pcimt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pcimt_realize;
    device_class_set_legacy_reset(dc, pcimt_reset);
    device_class_set_props(dc, pcimt_props);
    dc->user_creatable = false;
}

static const TypeInfo pcimt_info = {
    .name = TYPE_SNI_PCIMT,
    .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(SniPcimtState),
    .class_init = pcimt_class_init,
};

/* Intel 82375EB PCEB PCI-EISA bridge, config space only */
#define TYPE_SNI_PCEB "i82375-pceb"

static void pceb_realize(PCIDevice *d, Error **errp)
{
}

static void pceb_class_init(ObjectClass *klass, const void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = pceb_realize;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = 0x0482;
    k->revision = 0x05;
    k->class_id = 0x0000;
    dc->user_creatable = false;
}

static const TypeInfo pceb_info = {
    .name = TYPE_SNI_PCEB,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCIDevice),
    .class_init = pceb_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

/* ESC config (0x22/0x23) and SuperIO config (0x2e/0x2f) index/data pairs */
typedef struct IdxRegs {
    const char *name;
    uint8_t idx;
    uint8_t regs[256];
} IdxRegs;

static uint64_t idx_read(void *opaque, hwaddr addr, unsigned size)
{
    IdxRegs *r = opaque;
    if (!addr) {
        return r->idx;
    }
    trace_sni_idx_read(r->name, r->idx, r->regs[r->idx]);
    return r->regs[r->idx];
}

static void idx_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IdxRegs *r = opaque;

    if (!addr) {
        r->idx = val;
        return;
    }
    trace_sni_idx_write(r->name, r->idx, val);
    r->regs[r->idx] = val;
}

static const MemoryRegionOps idx_ops = {
    .read = idx_read,
    .write = idx_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
};

static void add_idx_regs(MemoryRegion *io, hwaddr base, const char *name)
{
    IdxRegs *r = g_new0(IdxRegs, 1);
    MemoryRegion *mr = g_new(MemoryRegion, 1);

    r->name = name;
    memory_region_init_io(mr, NULL, &idx_ops, r, name, 2);
    memory_region_add_subregion(io, base, mr);
}

static MemTxResult unassigned_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "bus error: read  0x%09" HWADDR_PRIx
                  "/%d\n", addr, size);
    *data = ~0ull;
    return MEMTX_DECODE_ERROR;
}

static MemTxResult unassigned_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size, MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "bus error: write 0x%09" HWADDR_PRIx
                  "/%d = 0x%" PRIx64 "\n", addr, size, data);
    return MEMTX_DECODE_ERROR;
}

static const MemoryRegionOps unassigned_ops = {
    .read_with_attrs = unassigned_read,
    .write_with_attrs = unassigned_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void main_cpu_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void load_prom(MachineState *machine, MemoryRegion *prom)
{
    uint8_t *p = memory_region_get_ram_ptr(prom);
    g_autofree char *filename = NULL;
    g_autofree uint8_t *buf = NULL;
    gsize len;
    int i;

    if (!machine->firmware) {
        if (!qtest_enabled()) {
            error_report("no PROM image given, use -bios");
            exit(1);
        }
        return;
    }
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!filename || !g_file_get_contents(filename, (char **)&buf, &len,
                                          NULL)) {
        error_report("could not load PROM '%s'", machine->firmware);
        exit(1);
    }
    if (len > PROM_SIZE) {
        len = PROM_SIZE;
    }
    memset(p, 0xff, PROM_SIZE);
    memcpy(p, buf, len);

    /*
     * Flash dumps hold 32-bit words in little-endian byte order. A big-endian
     * CPU fetches them unchanged, so swap if the reset vector reads as "j".
     */
    if (TARGET_BIG_ENDIAN && (p[3] >> 2) == 2) {
        for (i = 0; i < PROM_SIZE; i += 4) {
            stl_be_p(p + i, ldl_le_p(p + i));
        }
    }
}

/*
 * Memory lives at 0x20000000: 4 banks 128MB apart, each with two 64MB sides.
 * Unpopulated space reads as all-ones without a bus error (the PROM sizes
 * memory by probing; 5.02xx PROMs also probe the larger SIMM layouts up
 * to 4GB). The first 256MB are also visible at 0.
 */
#define MEM_BASE        0x20000000
#define MEM_SPACE       (512 * MiB)
#define MEM_SIDE        (64 * MiB)
#define MEM_LOW         (256 * MiB)
#define MEM_TOP         0x100000000ull

static uint64_t nomem_read(void *opaque, hwaddr addr, unsigned size)
{
    return ~0ull;
}

static void nomem_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
}

static const MemoryRegionOps nomem_ops = {
    .read = nomem_read,
    .write = nomem_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void rm200_init_memory(MachineState *machine, MemoryRegion *sysmem)
{
    MemoryRegion *space = g_new(MemoryRegion, 1);
    MemoryRegion *nomem = g_new(MemoryRegion, 1);
    MemoryRegion *nomem_hi = g_new(MemoryRegion, 1);
    MemoryRegion *low = g_new(MemoryRegion, 1);
    uint64_t left = machine->ram_size;
    uint64_t off = 0;
    int side;

    if (left > MEM_SPACE || left % (16 * MiB)) {
        error_report("RAM size must be a multiple of 16MB, at most 512MB");
        exit(1);
    }
    memory_region_init(space, NULL, "rm200.memspace", MEM_SPACE);
    memory_region_init_io(nomem, NULL, &nomem_ops, NULL, "rm200.nomem",
                          MEM_SPACE);
    memory_region_add_subregion_overlap(space, 0, nomem, -1);
    for (side = 0; side < MEM_SPACE / MEM_SIDE && left; side++) {
        uint64_t sz = pow2floor(MIN(left, MEM_SIDE));
        MemoryRegion *mr = g_new(MemoryRegion, 1);

        memory_region_init_alias(mr, NULL, "rm200.dimm", machine->ram, off,
                                 sz);
        memory_region_add_subregion(space, side * MEM_SIDE, mr);
        off += sz;
        left -= sz;
    }
    memory_region_add_subregion(sysmem, MEM_BASE, space);
    memory_region_init_io(nomem_hi, NULL, &nomem_ops, NULL, "rm200.nomem-hi",
                          MEM_TOP - MEM_BASE - MEM_SPACE);
    memory_region_add_subregion_overlap(sysmem, MEM_BASE + MEM_SPACE,
                                        nomem_hi, -1);
    memory_region_init_alias(low, NULL, "rm200.lowmem", space, 0, MEM_LOW);
    memory_region_add_subregion(sysmem, 0, low);
}

/*
 * IDPROM in the first 2KB of NVRAM. 0x22 = 0xff selects the long format,
 * 0x50 motherboard module number, 0x8b motherboard serial number (18
 * alphanumerics, checked by POST), 0x7ff makes the 2KB sum to zero.
 */
#define IDPROM_SIZE     0x800

static void rm200_init_idprom(uint8_t *p)
{
    uint8_t sum = 0;
    int i;

    if (!buffer_is_zero(p, NVRAM_SIZE)) {
        return;
    }
    p[0x22] = 0xff;
    memcpy(p + 0x50, "QEMU RM200C", 11);
    memcpy(p + 0x8b, "QEMU00000000000001", 18);
    for (i = 0; i < IDPROM_SIZE - 1; i++) {
        sum += p[i];
    }
    p[IDPROM_SIZE - 1] = -sum;
}

static void rm200_init_nvram(const char *path, MemoryRegion *mr)
{
    int fd;

    if (!path) {
        memory_region_init_ram(mr, NULL, "rm200.nvram", NVRAM_SIZE,
                               &error_fatal);
        rm200_init_idprom(memory_region_get_ram_ptr(mr));
        return;
    }
    fd = qemu_create(path, O_RDWR | O_BINARY, 0644, &error_fatal);
    if (ftruncate(fd, NVRAM_SIZE) < 0) {
        error_report("could not size NVRAM file '%s'", path);
        exit(1);
    }
    memory_region_init_ram_from_fd(mr, NULL, "rm200.nvram", NVRAM_SIZE,
                                   RAM_SHARED, fd, 0, &error_fatal);
    rm200_init_idprom(memory_region_get_ram_ptr(mr));
}

static void sni_rm200_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *prom = g_new(MemoryRegion, 1);
    MemoryRegion *nvram = g_new(MemoryRegion, 1);
    MemoryRegion *nvram_alias;
    MemoryRegion *cache_repl = g_new(MemoryRegion, 1);
    MemoryRegion *unassigned = g_new(MemoryRegion, 1);
    DeviceState *dev;
    SysBusDevice *sbd;
    SniPcimtState *pcimt;
    PCIBus *pci_bus;
    ISABus *isa_bus;
    ISADevice *pit, *isadev;
    qemu_irq *i8259;
    DriveInfo *fds[MAX_FD];
    Clock *cpuclk;
    MIPSCPU *cpu;
    CPUMIPSState *env;
    int i;

    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 133333333);
    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk,
                                     TARGET_BIG_ENDIAN);
    env = &cpu->env;
    qemu_register_reset(main_cpu_reset, cpu);
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);

    memory_region_init_io(unassigned, NULL, &unassigned_ops, NULL,
                          "unassigned", 1ull << 36);
    memory_region_add_subregion_overlap(sysmem, 0, unassigned, -1000);

    rm200_init_memory(machine, sysmem);

    memory_region_init_rom(prom, NULL, "rm200.prom", PROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, PROM_BASE, prom);
    load_prom(machine, prom);

    rm200_init_nvram(SNI_RM200_MACHINE(machine)->nvram_file, nvram);
    memory_region_add_subregion(sysmem, NVRAM_BASE, nvram);
    for (i = 1; i < 512 * KiB / NVRAM_SIZE; i++) {
        nvram_alias = g_new(MemoryRegion, 1);
        memory_region_init_alias(nvram_alias, NULL, "rm200.nvram-mirror",
                                 nvram, 0, NVRAM_SIZE);
        memory_region_add_subregion(sysmem, NVRAM_BASE + i * NVRAM_SIZE,
                                    nvram_alias);
    }

    memory_region_init_ram(cache_repl, NULL, "rm200.cache-repl",
                           CACHE_REPL_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CACHE_REPL_BASE, cache_repl);

    /* ASIC PCI, board registers, PCI host */
    dev = qdev_new(TYPE_SNI_PCIMT);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    pcimt = SNI_PCIMT(dev);
    sysbus_mmio_map(sbd, 0, RM_ISA_MEM_BASE);
    sysbus_mmio_map(sbd, 1, RM_PCI_MEM_BASE);
    sysbus_mmio_map(sbd, 2, RM_PCI_IO_BASE);
    sysbus_mmio_map(sbd, 3, ASIC_BASE);
    sysbus_mmio_map(sbd, 4, XBUS_BASE);
    sysbus_mmio_map_overlap(sbd, 5, INTACK_BASE, 1);
    for (i = 0; i < 5; i++) {
        sysbus_connect_irq(sbd, i, env->irq[2 + i]);
    }
    pci_bus = PCI_HOST_BRIDGE(dev)->bus;

    /* EISA side of the Intel 82374EB ESC */
    isa_bus = isa_bus_new(NULL, &pcimt->pci_mem, &pcimt->pci_io,
                          &error_abort);
    i8259 = i8259_init(isa_bus, qdev_get_gpio_in(dev, 5));
    isa_bus_register_input_irqs(isa_bus, i8259);
    i8257_dma_init(OBJECT(machine), isa_bus, 0);
    pit = i8254_pit_init(isa_bus, 0x40, 0, NULL);
    isadev = isa_new(TYPE_PC_SPEAKER);
    object_property_set_link(OBJECT(isadev), "pit", OBJECT(pit), &error_fatal);
    isa_realize_and_unref(isadev, isa_bus, &error_fatal);
    mc146818_rtc_init(isa_bus, 1900, NULL);
    add_idx_regs(&pcimt->pci_io, 0x22, "esc-cfg");

    /* National SuperIO: UARTs, LPT, FDC; 8042 keyboard controller */
    add_idx_regs(&pcimt->pci_io, 0x2e, "superio-cfg");
    for (i = 0; i < 2; i++) {
        isadev = isa_new(TYPE_ISA_SERIAL);
        qdev_prop_set_uint32(DEVICE(isadev), "index", i);
        qdev_prop_set_chr(DEVICE(isadev), "chardev", serial_hd(i));
        isa_realize_and_unref(isadev, isa_bus, &error_fatal);
    }
    isadev = isa_new(TYPE_ISA_PARALLEL);
    qdev_prop_set_uint32(DEVICE(isadev), "iobase", 0x3bc);
    qdev_prop_set_chr(DEVICE(isadev), "chardev", parallel_hds[0] ?:
                      qemu_chr_new("rm200.lpt", "null", NULL));
    isa_realize_and_unref(isadev, isa_bus, &error_fatal);
    isa_create_simple(isa_bus, TYPE_I8042);
    for (i = 0; i < MAX_FD; i++) {
        fds[i] = drive_get(IF_FLOPPY, 0, i);
    }
    isadev = isa_new(TYPE_ISA_FDC);
    isa_realize_and_unref(isadev, isa_bus, &error_fatal);
    isa_fdc_init_drives(isadev, fds);

    /* PCI devices */
    pci_create_simple(pci_bus, PCI_DEVFN(0, 0), TYPE_SNI_PCEB);
    dev = DEVICE(pci_create_simple(pci_bus, PCI_DEVFN(1, 0), "lsi53c810"));
    lsi53c8xx_handle_legacy_cmdline(dev);
    pci_init_nic_in_slot(pci_bus, "pcnet", NULL, "2");
    pci_create_simple(pci_bus, PCI_DEVFN(3, 0), "cirrus-vga-gd5434");
    pci_init_nic_devices(pci_bus, "pcnet");
}

static char *rm200_get_nvram(Object *obj, Error **errp)
{
    return g_strdup(SNI_RM200_MACHINE(obj)->nvram_file);
}

static void rm200_set_nvram(Object *obj, const char *value, Error **errp)
{
    SniRm200MachineState *s = SNI_RM200_MACHINE(obj);

    g_free(s->nvram_file);
    s->nvram_file = g_strdup(value);
}

/* No x86 option ROMs; SINIX sets drive mode pages QEMU cannot apply */
static GlobalProperty rm200_props[] = {
    { "pcnet", "romfile", "" },
    { "cirrus-vga", "romfile", "" },  /* also covers cirrus-vga-gd5434 */
    { "VGA", "romfile", "" },
    { "scsi-hd", "quirk_mode_select_ignore", "on" },
    { "scsi-cd", "quirk_mode_select_ignore", "on" },
};

static void sni_rm200_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    compat_props_add(mc->compat_props, rm200_props, ARRAY_SIZE(rm200_props));
    mc->desc = "Siemens Nixdorf RM200C (PCI)";
    mc->default_nic = "pcnet";
    mc->init = sni_rm200_init;
    mc->block_default_type = IF_SCSI;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R4700");
    mc->default_ram_id = "rm200.ram";
    mc->default_ram_size = 64 * MiB;
    object_class_property_add_str(oc, "nvram", rm200_get_nvram,
                                  rm200_set_nvram);
    object_class_property_set_description(oc, "nvram",
                                          "File backing the 32KB NVRAM");
}

static const TypeInfo sni_rm200_type = {
    .name = TYPE_SNI_RM200_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(SniRm200MachineState),
    .class_init = sni_rm200_class_init,
};

static void sni_rm200_register_types(void)
{
    type_register_static(&pcimt_info);
    type_register_static(&pceb_info);
    type_register_static(&sni_rm200_type);
}

type_init(sni_rm200_register_types)
