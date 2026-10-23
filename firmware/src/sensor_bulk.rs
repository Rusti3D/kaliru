// Helper code for collecting and sending bulk sensor measurements
//
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::sched::R;
use crate::sendf;

#[repr(C)]
pub struct SensorBulk {
    pub sequence: u16,
    pub possible_overflows: u16,
    pub data_count: u8,
    pub data: [u8; 51],
}

// Reset counters
pub fn sensor_bulk_reset(sb: &mut SensorBulk) {
    sb.sequence = 0;
    sb.possible_overflows = 0;
    sb.data_count = 0;
}

// Report local measurement buffer
pub fn sensor_bulk_report(sb: &mut SensorBulk, oid: u8) -> R {
    sendf!("sensor_bulk_data oid=%c sequence=%hu data=%*s", oid, sb.sequence, &sb.data[..sb.data_count as usize])?;
    sb.data_count = 0;
    sb.sequence = sb.sequence.wrapping_add(1);
    Ok(())
}

// Report buffer and fifo status
pub fn sensor_bulk_status(sb: &SensorBulk, oid: u8, time1: u32, query_ticks: u32, fifo: u32) -> R {
    sendf!(
        concat!("sensor_bulk_status oid=%c clock=%u query_ticks=%u next_sequence=%hu", " buffered=%u possible_overflows=%hu"),
        oid,
        time1,
        query_ticks,
        sb.sequence,
        (sb.data_count as u32).wrapping_add(fifo),
        sb.possible_overflows
    )
}
