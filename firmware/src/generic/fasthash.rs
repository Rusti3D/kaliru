// The fast-hash function (lib/fast-hash/fasthash.c)
//
// The MIT License
// Copyright (C) 2012 Zilong Tan (eric.zltan@gmail.com)
// (see Kalico lib/fast-hash for the full license text)

// Compression function for Merkle-Damgard construction.
// This function is generated using the framework provided.
fn mix(mut h: u64) -> u64 {
    h ^= h >> 23;
    h = h.wrapping_mul(0x2127599bf4325c37);
    h ^= h >> 47;
    h
}

pub fn fasthash64(buf: &[u8], seed: u64) -> u64 {
    let m: u64 = 0x880355f21e6d1965;
    let len = buf.len();
    let mut h = seed ^ (len as u64).wrapping_mul(m);

    let mut chunks = buf.chunks_exact(8);
    for c in &mut chunks {
        // (the C code reads native endian uint64_t words)
        let v = u64::from_ne_bytes(c.try_into().unwrap());
        h ^= mix(v);
        h = h.wrapping_mul(m);
    }

    let pos2 = chunks.remainder();
    let mut v: u64 = 0;
    if !pos2.is_empty() {
        for (i, b) in pos2.iter().enumerate() {
            v ^= (*b as u64) << (8 * i);
        }
        h ^= mix(v);
        h = h.wrapping_mul(m);
    }

    mix(h)
}

#[allow(dead_code)]
pub fn fasthash32(buf: &[u8], seed: u32) -> u32 {
    // the following trick converts the 64-bit hashcode to Fermat
    // residue, which shall retain information from both the higher
    // and lower parts of hashcode.
    let h = fasthash64(buf, seed as u64);
    h.wrapping_sub(h >> 32) as u32
}
