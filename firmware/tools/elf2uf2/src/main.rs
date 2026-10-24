// Kalico's lib/elf2uf2 (main.cpp, elf.h; the uf2 constants of
// lib/pico-sdk/boot/uf2.h): the klipper.uf2 of an rp2040 / rp2350 build.
//
// Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
//
// SPDX-License-Identifier: BSD-3-Clause
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};

const ERROR_ARGS: i32 = -1;
const ERROR_FORMAT: i32 = -2;
const ERROR_INCOMPATIBLE: i32 = -3;
const ERROR_READ_FAILED: i32 = -4;
const ERROR_WRITE_FAILED: i32 = -5;

// elf.h
const ELF_MAGIC: u32 = 0x464c457f;
const EM_ARM: u16 = 0x28;
const EF_ARM_ABI_FLOAT_HARD: u32 = 0x00000400;
const PT_LOAD: u32 = 0x00000001;
const ELF32_HEADER_SIZE: usize = 52;
const ELF32_PH_ENTRY_SIZE: usize = 32;

// boot/uf2.h
const UF2_MAGIC_START0: u32 = 0x0A324655;
const UF2_MAGIC_START1: u32 = 0x9E5D5157;
const UF2_MAGIC_END: u32 = 0x0AB16F30;
const UF2_FLAG_FAMILY_ID_PRESENT: u32 = 0x00002000;
const RP2040_FAMILY_ID: u32 = 0xe48bff56;
const UF2_DATA_SIZE: usize = 476;

// we require 256 (as this is the page size supported by the device)
const LOG2_PAGE_SIZE: u32 = 8;
const PAGE_SIZE: u32 = 1 << LOG2_PAGE_SIZE;

#[derive(Clone, Copy, PartialEq)]
enum RangeType {
    Contents,   // may have contents
    NoContents, // must be uninitialized
    Ignore,     // will be ignored
}

#[derive(Clone, Copy)]
struct AddressRange {
    from: u32,
    to: u32,
    typ: RangeType,
}

const fn range(from: u32, to: u32, typ: RangeType) -> AddressRange {
    AddressRange { from, to, typ }
}

const MAIN_RAM_START: u32 = 0x20000000;
const MAIN_RAM_END: u32 = 0x20042000;
const FLASH_START: u32 = 0x10000000;
const FLASH_END: u32 = 0x15000000;
const XIP_SRAM_START: u32 = 0x15000000;
const XIP_SRAM_END: u32 = 0x15004000;
const MAIN_RAM_BANKED_START: u32 = 0x21000000;
const MAIN_RAM_BANKED_END: u32 = 0x21040000;

const RP2040_ADDRESS_RANGES_FLASH: [AddressRange; 3] = [
    range(FLASH_START, FLASH_END, RangeType::Contents),
    range(MAIN_RAM_START, MAIN_RAM_END, RangeType::NoContents),
    range(MAIN_RAM_BANKED_START, MAIN_RAM_BANKED_END, RangeType::NoContents),
];

const RP2040_ADDRESS_RANGES_RAM: [AddressRange; 3] = [
    range(MAIN_RAM_START, MAIN_RAM_END, RangeType::Contents),
    range(XIP_SRAM_START, XIP_SRAM_END, RangeType::Contents),
    range(0x00000000, 0x00004000, RangeType::Ignore), // for now we ignore the bootrom if present
];

struct PageFragment {
    file_offset: u32,
    page_offset: u32,
    bytes: u32,
}

struct Elf32Header {
    entry: u32,
    ph_entry_size: u16,
    ph_num: u16,
}

struct Elf2uf2 {
    error_msg: String,
    verbose: bool,
}

fn u16_at(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

impl Elf2uf2 {
    fn fail(&mut self, code: i32, msg: String) -> i32 {
        self.error_msg = msg;
        code
    }

    fn fail_read_error(&mut self) -> i32 {
        self.fail(ERROR_READ_FAILED, "Failed to read input file".into())
    }

    fn fail_write_error(&mut self) -> i32 {
        self.fail(ERROR_WRITE_FAILED, "Failed to write output file".into())
    }

    fn read_and_check_elf32_header(&mut self, inp: &mut File) -> Result<Elf32Header, i32> {
        let mut b = [0u8; ELF32_HEADER_SIZE];
        if inp.read_exact(&mut b).is_err() {
            return Err(self.fail(ERROR_READ_FAILED, "Unable to read ELF header".into()));
        }
        let (magic, arch_class, endianness, version, abi) = (u32_at(&b, 0), b[4], b[5], b[6], b[7]);
        let (machine, version2) = (u16_at(&b, 18), u32_at(&b, 20));
        let (flags, eh_size) = (u32_at(&b, 36), u16_at(&b, 40));
        if magic != ELF_MAGIC {
            return Err(self.fail(ERROR_FORMAT, "Not an ELF file".into()));
        }
        if version != 1 || version2 != 1 {
            return Err(self.fail(ERROR_FORMAT, "Unrecognized ELF version".into()));
        }
        if arch_class != 1 || endianness != 1 {
            return Err(self.fail(ERROR_INCOMPATIBLE, "Require 32 bit little-endian ELF".into()));
        }
        if eh_size as usize != ELF32_HEADER_SIZE {
            return Err(self.fail(ERROR_FORMAT, "Invalid ELF32 format".into()));
        }
        if machine != EM_ARM {
            return Err(self.fail(ERROR_FORMAT, "Not an ARM executable".into()));
        }
        if abi != 0 {
            return Err(self.fail(ERROR_INCOMPATIBLE, "Unrecognized ABI".into()));
        }
        if flags & EF_ARM_ABI_FLOAT_HARD != 0 {
            return Err(self.fail(ERROR_INCOMPATIBLE, "HARD-FLOAT not supported".into()));
        }
        Ok(Elf32Header { entry: u32_at(&b, 24), ph_entry_size: u16_at(&b, 42), ph_num: u16_at(&b, 44) })
    }

    fn check_address_range(
        &mut self,
        valid_ranges: &[AddressRange],
        addr: u32,
        vaddr: u32,
        size: u32,
        uninitialized: bool,
    ) -> Result<AddressRange, i32> {
        for range in valid_ranges {
            if range.from <= addr && range.to >= addr.wrapping_add(size) {
                if range.typ == RangeType::NoContents && !uninitialized {
                    return Err(self.fail(ERROR_INCOMPATIBLE, "ELF contains memory contents for uninitialized memory".into()));
                }
                if self.verbose {
                    println!(
                        "{} segment {:08x}->{:08x} ({:08x}->{:08x})",
                        if uninitialized { "Uninitialized" } else { "Mapped" },
                        addr,
                        addr.wrapping_add(size),
                        vaddr,
                        vaddr.wrapping_add(size)
                    );
                }
                return Ok(*range);
            }
        }
        Err(self.fail(
            ERROR_INCOMPATIBLE,
            format!("Memory segment {:08x}->{:08x} is outside of valid address range for device", addr, addr.wrapping_add(size)),
        ))
    }

    fn read_and_check_elf32_ph_entries(
        &mut self,
        inp: &mut File,
        eh: &Elf32Header,
        valid_ranges: &[AddressRange],
        pages: &mut BTreeMap<u32, Vec<PageFragment>>,
    ) -> i32 {
        if eh.ph_entry_size as usize != ELF32_PH_ENTRY_SIZE {
            return self.fail(ERROR_FORMAT, "Invalid ELF32 program header".into());
        }
        if eh.ph_num != 0 {
            // (C reads the entries right after the header, not at its ph_offset)
            let mut b = vec![0u8; eh.ph_num as usize * ELF32_PH_ENTRY_SIZE];
            if inp.read_exact(&mut b).is_err() {
                return self.fail_read_error();
            }
            for i in 0..eh.ph_num as usize {
                let e = &b[i * ELF32_PH_ENTRY_SIZE..];
                let (typ, offset, vaddr, paddr, filez, memsz) =
                    (u32_at(e, 0), u32_at(e, 4), u32_at(e, 8), u32_at(e, 12), u32_at(e, 16), u32_at(e, 20));
                if typ == PT_LOAD && memsz != 0 {
                    let mapped_size = filez.min(memsz);
                    if mapped_size != 0 {
                        let ar = match self.check_address_range(valid_ranges, paddr, vaddr, mapped_size, false) {
                            Ok(ar) => ar,
                            Err(rc) => return rc,
                        };
                        // we don't download uninitialized, generally it is BSS and should be zero-ed by crt0.S, or it may be COPY areas which are undefined
                        if ar.typ != RangeType::Contents {
                            if self.verbose {
                                println!("  ignored");
                            }
                            continue;
                        }
                        let mut addr = paddr;
                        let mut remaining = mapped_size;
                        let mut file_offset = offset;
                        while remaining != 0 {
                            let off = addr & (PAGE_SIZE - 1);
                            let len = remaining.min(PAGE_SIZE - off);
                            let fragments = pages.entry(addr - off).or_default(); // list of fragments
                            // note if filesz is zero, we want zero init which is handled because the
                            // statement above creates an empty page fragment list
                            // check overlap with any existing fragments
                            let overlap = fragments
                                .iter()
                                .any(|f| (off < f.page_offset + f.bytes) != ((off + len) <= f.page_offset));
                            fragments.push(PageFragment { file_offset, page_offset: off, bytes: len });
                            if overlap {
                                // (C notes the error and goes on)
                                self.fail(ERROR_FORMAT, "In memory segments overlap".into());
                            }
                            addr = addr.wrapping_add(len);
                            file_offset = file_offset.wrapping_add(len);
                            remaining -= len;
                        }
                    }
                    if memsz > filez {
                        // we have some uninitialized data too
                        if let Err(rc) = self.check_address_range(
                            valid_ranges,
                            paddr.wrapping_add(filez),
                            vaddr.wrapping_add(filez),
                            memsz - filez,
                            true,
                        ) {
                            return rc;
                        }
                    }
                }
            }
        }
        0
    }

    fn realize_page(&mut self, inp: &mut File, fragments: &[PageFragment], buf: &mut [u8]) -> i32 {
        assert!(buf.len() >= PAGE_SIZE as usize);
        for frag in fragments {
            assert!(frag.page_offset < PAGE_SIZE && frag.page_offset + frag.bytes <= PAGE_SIZE);
            if inp.seek(SeekFrom::Start(frag.file_offset as u64)).is_err() {
                return self.fail_read_error();
            }
            let (o, n) = (frag.page_offset as usize, frag.bytes as usize);
            if inp.read_exact(&mut buf[o..o + n]).is_err() {
                return self.fail_read_error();
            }
        }
        0
    }

    fn elf2uf2(&mut self, inp: &mut File, out: &mut File) -> i32 {
        let mut pages: BTreeMap<u32, Vec<PageFragment>> = BTreeMap::new();
        let eh = match self.read_and_check_elf32_header(inp) {
            Ok(eh) => eh,
            Err(rc) => return rc,
        };
        let ram_style = is_address_initialized(&RP2040_ADDRESS_RANGES_RAM, eh.entry);
        if self.verbose {
            if ram_style {
                println!("Detected RAM binary");
            } else {
                println!("Detected FLASH binary");
            }
        }
        let valid_ranges: &[AddressRange] = if ram_style { &RP2040_ADDRESS_RANGES_RAM } else { &RP2040_ADDRESS_RANGES_FLASH };
        let rc = self.read_and_check_elf32_ph_entries(inp, &eh, valid_ranges, &mut pages);
        if rc != 0 {
            return rc;
        }
        if pages.is_empty() {
            return self.fail(ERROR_INCOMPATIBLE, "The input file has no memory pages".into());
        }
        if ram_style {
            let mut expected_ep_main_ram = u32::MAX;
            let mut expected_ep_xip_sram = u32::MAX;
            for &page in pages.keys() {
                if (MAIN_RAM_START..MAIN_RAM_END).contains(&page) && page < expected_ep_main_ram {
                    expected_ep_main_ram = page | 0x1;
                } else if (XIP_SRAM_START..XIP_SRAM_END).contains(&page) && page < expected_ep_xip_sram {
                    expected_ep_xip_sram = page | 0x1;
                }
            }
            let expected_ep = if u32::MAX != expected_ep_main_ram { expected_ep_main_ram } else { expected_ep_xip_sram };
            if eh.entry == expected_ep_xip_sram {
                return self.fail(ERROR_INCOMPATIBLE, "B0/B1 Boot ROM does not support direct entry into XIP_SRAM\n".into());
            } else if eh.entry != expected_ep {
                return self.fail(
                    ERROR_INCOMPATIBLE,
                    format!("A RAM binary should have an entry point at the beginning: {:08x} (not {:08x})\n", expected_ep, eh.entry),
                );
            }
        }
        let num_blocks = pages.len() as u32;
        for (page_num, (&target_addr, fragments)) in pages.iter().enumerate() {
            if self.verbose {
                println!("Page {} / {} {:08x}", page_num, num_blocks, target_addr);
            }
            let mut data = [0u8; UF2_DATA_SIZE];
            let rc = self.realize_page(inp, fragments, &mut data);
            if rc != 0 {
                return rc;
            }
            // struct uf2_block
            let mut block = Vec::with_capacity(512);
            for v in [
                UF2_MAGIC_START0,
                UF2_MAGIC_START1,
                UF2_FLAG_FAMILY_ID_PRESENT,
                target_addr,
                PAGE_SIZE,
                page_num as u32,
                num_blocks,
                RP2040_FAMILY_ID,
            ] {
                block.extend_from_slice(&v.to_le_bytes());
            }
            block.extend_from_slice(&data);
            block.extend_from_slice(&UF2_MAGIC_END.to_le_bytes());
            if out.write_all(&block).is_err() {
                return self.fail_write_error();
            }
        }
        0
    }
}

fn is_address_initialized(valid_ranges: &[AddressRange], addr: u32) -> bool {
    for range in valid_ranges {
        if range.from <= addr && range.to > addr {
            return RangeType::Contents == range.typ;
        }
    }
    false
}

fn usage() -> i32 {
    eprintln!("Usage: elf2uf2 (-v) <input ELF file> <output UF2 file>");
    ERROR_ARGS
}

fn run() -> i32 {
    let argv: Vec<String> = std::env::args().collect();
    let mut st = Elf2uf2 { error_msg: String::new(), verbose: false };
    let mut arg = 1;
    if arg < argv.len() && argv[arg] == "-v" {
        st.verbose = true;
        arg += 1;
    }
    if argv.len() < arg + 2 {
        return usage();
    }
    let in_filename = &argv[arg];
    let Ok(mut inp) = File::open(in_filename) else {
        eprintln!("Can't open input file '{}'", in_filename);
        return ERROR_ARGS;
    };
    let out_filename = &argv[arg + 1];
    let Ok(mut out) = File::create(out_filename) else {
        eprintln!("Can't open output file '{}'", out_filename);
        return ERROR_ARGS;
    };

    let rc = st.elf2uf2(&mut inp, &mut out);
    drop(out);
    if rc != 0 {
        let _ = std::fs::remove_file(out_filename);
        if !st.error_msg.is_empty() {
            eprintln!("ERROR: {}", st.error_msg);
        }
    }
    rc
}

fn main() {
    std::process::exit(run());
}
