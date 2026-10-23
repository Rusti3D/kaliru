// Code for crc16_ccitt
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// Implement the standard crc "ccitt" algorithm on the given buffer
pub fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in buf {
        let mut data = b;
        data ^= (crc & 0xff) as u8;
        data ^= data << 4;
        crc = (((data as u16) << 8) | (crc >> 8)) ^ (data >> 4) as u16 ^ ((data as u16) << 3);
    }
    crc
}
