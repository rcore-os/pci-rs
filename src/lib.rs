/* Copyright (c) 2015 The Robigalia Project Developers
 * Licensed under the Apache License, Version 2.0
 * <LICENSE-APACHE or
 * http://www.apache.org/licenses/LICENSE-2.0> or the MIT
 * license <LICENSE-MIT or http://opensource.org/licenses/MIT>,
 * at your option. All files in the project carrying such
 * notice may not be copied, modified, or distributed except
 * according to those terms.
 */

#![feature(no_std)]
#![no_std]

//! PCI bus management
//!
//! This crate defines various traits, functions, and types for working with the PCI local bus.
//!
//!
//! It is assumed that PCI(e) is already configured - that is, that each device has been allocated
//! the memory it requests and the BARs are already configured correctly. The firmware (BIOS, UEFI)
//! usually does this on PC platforms.
//!
//! This crate is not yet suitable for multicore use - nothing is synchronized.
//!
//! This crate does not yet contain any hardware-specific workarounds for buggy or broken hardware.
//!
//! This crate cannot yet exploit PCIe memory-mapped configuration spaces.
//!
//! This crate only supports x86, currently.

/// A trait defining port I/O operations.
///
/// All port I/O operations are parametric over this trait. This allows operating systems to use
/// this crate without modifications, by suitably instantiating this trait with their own
/// primitives.
pub trait PortOps {
    unsafe fn read8(&self, port: u16) -> u8;
    unsafe fn read16(&self, port: u16) -> u16;
    unsafe fn read32(&self, port: u16) -> u32;

    unsafe fn write8(&self, port: u16, val: u8);
    unsafe fn write16(&self, port: u16, val: u16);
    unsafe fn write32(&self, port: u16, val: u32);
}

const CONFIG_ADDRESS: u16 = 0x0CF8;
const CONFIG_DATA: u16 = 0x0CFC;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CSpaceAccessMethod {
    // The legacy, deprecated (as of PCI 2.0) IO-range method.
    // Until/unless there is a relevant platform that requires this, leave it out.
    // IO_Mechanism_2
    /// The legacy (pre-PCIe) 2-IO port method as specified on page 50 of PCI Local Bus
    /// Specification 3.0.
    IO,
    // PCIe memory-mapped configuration space access
    //MemoryMapped(*mut u8),
}

// All IO-bus ops are 32-bit, we mask and shift to get the values we want.

impl CSpaceAccessMethod {
    pub unsafe fn read8<T: PortOps>(self, ops: &T, loc: Location, offset: u16) -> u8 {
        let val = self.read32(ops, loc, offset & 0b11111100);
        ((val >> ((offset as usize & 0b11) << 3)) & 0xFF) as u8
    }

    /// Returns a value in native endian.
    pub unsafe fn read16<T: PortOps>(self, ops: &T, loc: Location, offset: u16) -> u16 {
        let val = self.read32(ops, loc, offset & 0b11111100);
        ((val >> ((offset as usize & 0b10) << 3)) & 0xFFFF) as u16
    }

    /// Returns a value in native endian.
    pub unsafe fn read32<T: PortOps>(self, ops: &T, loc: Location, offset: u16) -> u32 {
        debug_assert!((offset & 0b11) == 0, "misaligned PCI configuration dword u32 read");
        match self {
            CSpaceAccessMethod::IO => {
                ops.write32(CONFIG_ADDRESS, loc.encode() | ((offset as u32) & 0b11111100));
                ops.read32(CONFIG_DATA).to_le()
            },
            //MemoryMapped(ptr) => {
            //    // FIXME: Clarify whether the rules for GEP/GEPi forbid using regular .offset() here.
            //    ::core::intrinsics::volatile_load(::core::intrinsics::arith_offset(ptr, offset as usize))
            //}
        }
    }

    pub unsafe fn write8<T: PortOps>(self, ops: &T, loc: Location, offset: u16, val: u8) {
        let old = self.read32(ops, loc, offset);
        let dest = offset as usize & 0b11 << 3;
        let mask = (0xFF << dest) as u32;
        self.write32(ops, loc, offset, ((val as u32) << dest | (old & !mask)).to_le());
    }

    /// Converts val to little endian before writing.
    pub unsafe fn write16<T: PortOps>(self, ops: &T, loc: Location, offset: u16, val: u16) {
        let old = self.read32(ops, loc, offset);
        let dest = offset as usize & 0b10 << 3;
        let mask = (0xFFFF << dest) as u32;
        self.write32(ops, loc, offset, ((val as u32) << dest | (old & !mask)).to_le());
    }

    /// Takes a value in native endian, converts it to little-endian, and writes it to the PCI
    /// device configuration space at register `offset`.
    pub unsafe fn write32<T: PortOps>(self, ops: &T, loc: Location, offset: u16, val: u32) {
        debug_assert!((offset & 0b11) == 0, "misaligned PCI configuration dword u32 read");
        match self {
            CSpaceAccessMethod::IO => {
                ops.write32(CONFIG_ADDRESS, loc.encode() | (offset as u32 & 0b11111100));
                ops.write32(CONFIG_DATA, val.to_le())
            },
            //MemoryMapped(ptr) => {
            //    // FIXME: Clarify whether the rules for GEP/GEPi forbid using regular .offset() here.
            //    ::core::intrinsics::volatile_load(::core::intrinsics::arith_offset(ptr, offset as usize))
            //}
        }
    }
}

/// Physical location of a device on the bus
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Location {
    pub bus: u8,
    pub device: u8,
    pub function: u8,
}

impl Location {
    #[inline(always)]
    fn encode(self) -> u32 {
        (1 << 31) | ((self.bus as u32) << 16) | (((self.device as u32) & 0b11111) << 11) | (((self.function as u32) & 0b111) << 8)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Identifier {
    pub vendor_id: u16,
    pub device_id: u16,
    pub revision_id: u8,
    pub class: u8,
    pub subclass: u8,
}

/// A device on the PCI bus.
///
/// Although accessing configuration space may be expensive, it is not cached.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PCIDevice {
    pub loc: Location,
    pub id: Identifier,
    pub bars: [Option<BAR>; 6],
    pub cspace_access_method: CSpaceAccessMethod,
}

pub enum PCIScanError {

}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Prefetchable {
    Yes,
    No
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Type {
    Bits32,
    Bits64
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum BAR {
    Memory(u64, u32, Prefetchable, Type),
    IO(u32, u32),
}

impl BAR {
    pub unsafe fn decode<T: PortOps>(ops: &T, loc: Location, am: CSpaceAccessMethod, idx: u16) -> (Option<BAR>, usize) {
        let raw = am.read32(ops, loc, 16 + (idx << 2));
        if raw == 0 {
            return (None, idx as usize + 1);
        }
        if raw & 1 == 0 {
            let mut bits64 = false;
            let base: u64 =
            match (raw & 0b110) >> 1 {
                0 => (raw & !0xF) as u64,
                2 => { bits64 = true; ((raw & !0xF) as u64) | ((am.read32(ops, loc, 16 + ((idx + 1) << 2)) as u64) << 32) }
                _ => { debug_assert!(false, "bad type in memory BAR"); return (None, idx as usize + 1) },
            };
            am.write32(ops, loc, 16 + (idx << 2), !0);
            let len = !(am.read32(ops, loc, 16 + (idx << 2)) & !0xF) + 1;
            am.write32(ops, loc, 16 + (idx << 2), raw);
            (Some(BAR::Memory(base, len, if raw & 0b1000 == 0 { Prefetchable::No } else { Prefetchable::Yes },
                        if bits64 { Type::Bits64 } else { Type::Bits32 })),
             if bits64 { idx + 2 } else { idx + 1 } as usize)
        } else {
            am.write32(ops, loc, 16 + (idx << 2), !0);
            let len = !(am.read32(ops, loc, 16 + (idx << 2)) & !0x3) + 1;
            am.write32(ops, loc, 16 + (idx << 2), raw);
            (Some(BAR::IO(raw & !0x3, len)), idx as usize + 1)
        }
    }
}

pub struct BusScan<'a, T: PortOps+'a> {
    loc: Location,
    ops: &'a T,
    am: CSpaceAccessMethod,
}

impl<'a, T: PortOps> BusScan<'a, T> {
    fn done(&self) -> bool {
        if self.loc.bus == 255 && self.loc.device == 31 && self.loc.function == 7 {
            true
        } else {
            false
        }
    }

    fn increment(&mut self) {
        // TODO: Decide whether this is actually nicer than taking a u16 and incrementing until it
        // wraps.
        if self.loc.function < 7 {
            self.loc.function += 1;
            return
        } else {
            self.loc.function = 0;
            if self.loc.device < 31 {
                self.loc.device += 1;
                return;
            } else {
                self.loc.device = 0;
                if self.loc.bus == 255 {
                    self.loc.device = 31;
                    self.loc.device = 7;
                } else {
                    self.loc.bus += 1;
                    return;
                }
            }
        }
    }
}

impl<'a, T: PortOps> ::core::iter::Iterator for BusScan<'a, T> {
    type Item = PCIDevice;
    #[inline]
    fn next(&mut self) -> Option<PCIDevice> {
        // FIXME: very naive atm, could be smarter and waste much less time by only scanning used
        // busses.
        let mut ret = None;
        loop {
            if self.done() {
                return ret;
            }
            unsafe {
                ret = probe_function(self.ops, self.loc, self.am);
            }
            self.increment();
            if ret.is_some() {
                return ret;
            }
        }
    }
}

pub unsafe fn probe_function<T: PortOps>(ops: &T, loc: Location, am: CSpaceAccessMethod) -> Option<PCIDevice> {
    // FIXME: it'd be more efficient to use read32 and decode separately.
    let vid = am.read16(ops, loc, 0);
    if vid == 0xFFFF {
        return None;
    }
    let did = am.read16(ops, loc, 2);
    let rid = am.read8(ops, loc, 8);
    let subclass = am.read8(ops, loc, 10);
    let class = am.read8(ops, loc, 11);
    let id = Identifier {
        vendor_id: vid,
        device_id: did,
        revision_id: rid,
        class: class,
        subclass: subclass,
    };
    let hdrty = am.read8(ops, loc, 14);
    let mut bars = [None, None, None, None, None, None];
    let max = match hdrty {
        0 => 6,
        1 => 2,
        _ => 0,
    };
    let mut i = 0;
    while i < max {
        let (bar, next) = BAR::decode(ops, loc, am, i as u16);
        bars[i] = bar;
        i = next;
    }
    Some(PCIDevice {
        loc: loc,
        id: id,
        bars: bars,
        cspace_access_method: am,
    })
}

pub unsafe fn scan_bus<'a, T: PortOps>(ops: &'a T, am: CSpaceAccessMethod) -> BusScan<'a, T> {
    BusScan { loc: Location { bus: 0, device: 0, function: 0 }, ops: ops, am: am }
}
