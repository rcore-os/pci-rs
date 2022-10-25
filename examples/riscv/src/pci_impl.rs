use pci::{PortOps, CSpaceAccessMethod};

pub const PCI_COMMAND: u16 = 0x04;
pub const BAR0: u16 = 0x10;
pub const PCI_CAP_PTR: u16 = 0x34;
pub const PCI_INTERRUPT_LINE: u16 = 0x3c;
pub const PCI_INTERRUPT_PIN: u16 = 0x3d;
pub const PCI_COMMAND_INTX_DISABLE:u16 = 0x400;

pub const PCI_MSI_CTRL_CAP: u16 = 0x00;
pub const PCI_MSI_ADDR: u16 = 0x04;
pub const PCI_MSI_UPPER_ADDR: u16 = 0x08;
pub const PCI_MSI_DATA_32: u16 = 0x08;
pub const PCI_MSI_DATA_64: u16 = 0x0C;

pub const PCI_CAP_ID_MSI: u8 = 0x05;

pub struct PortOpsImpl;

cfg_if::cfg_if! {
    if #[cfg(target_arch = "x86_64")] {

use x86_64::instructions::port::Port;
impl PortOps for PortOpsImpl {
    unsafe fn read8(&self, port: u16) -> u8 {
        Port::new(port).read()
    }
    unsafe fn read16(&self, port: u16) -> u16 {
        Port::new(port).read()
    }
    unsafe fn read32(&self, port: u32) -> u32 {
        Port::new(port as u16).read()
    }
    unsafe fn write8(&self, port: u16, val: u8) {
        Port::new(port).write(val);
    }
    unsafe fn write16(&self, port: u16, val: u16) {
        Port::new(port).write(val);
    }
    unsafe fn write32(&self, port: u32, val: u32) {
        Port::new(port as u16).write(val);
    }
}

// fix me
pub const PCI_BASE: usize = 0;
pub const PCI_ACCESS: CSpaceAccessMethod = CSpaceAccessMethod::IO;

} else if #[cfg(target_arch = "riscv64")] {
use core::ptr::{read_volatile, write_volatile};

/// riscv64 qemu
pub const PCI_BASE: usize = 0x30000000;
pub const E1000_BASE: usize = 0x40000000;
pub const PCI_ACCESS: CSpaceAccessMethod = CSpaceAccessMethod::MemoryMapped(PCI_BASE as *mut u8);

pub fn phys_to_virt(paddr: usize) -> usize {
    paddr
}
pub fn virt_to_phys(vaddr: usize) -> usize {
    vaddr
}

#[inline(always)]
pub fn writev<T>(addr: usize, content: T) {
    let cell = (addr) as *mut T;
    unsafe {
        write_volatile(cell, content);
    }
}
#[inline(always)]
pub fn readv<T>(addr: usize) -> T {
    let cell = (addr) as *const T;
    unsafe { read_volatile(cell) }
}

impl PortOps for PortOpsImpl {
    unsafe fn read8(&self, port: u16) -> u8 {
        readv(PCI_BASE + port as usize)
    }
    unsafe fn read16(&self, port: u16) -> u16 {
        readv(PCI_BASE + port as usize)
    }
    unsafe fn read32(&self, port: u32) -> u32 {
        readv(PCI_BASE + port as usize)
    }
    unsafe fn write8(&self, port: u16, val: u8) {
        writev(PCI_BASE + port as usize, val);
    }
    unsafe fn write16(&self, port: u16, val: u16) {
        writev(PCI_BASE + port as usize, val);
    }
    unsafe fn write32(&self, port: u32, val: u32) {
        writev(PCI_BASE + port as usize, val);
    }
}

}
} // cfg_if
