// Trapezoidal velocity movement queue (port of klippy/chelper/trapq.c)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The C linked lists are VecDeques: `moves` runs oldest first between a
// head and a tail sentinel, `history` runs newest first (list_add_head).
// Code walking the move list uses a MoveRef (the list plus an index).
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Coord {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Coord {
    pub fn axis(&self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Move {
    pub print_time: f64,
    pub move_t: f64,
    pub start_v: f64,
    pub half_accel: f64,
    pub start_pos: Coord,
    pub axes_r: Coord,
}

impl Move {
    // Return the distance moved given a time in a move
    #[inline]
    pub fn get_distance(&self, move_time: f64) -> f64 {
        (self.start_v + self.half_accel * move_time) * move_time
    }

    // Return the XYZ coordinates given a time in a move
    #[inline]
    pub fn get_coord(&self, move_time: f64) -> Coord {
        let move_dist = self.get_distance(move_time);
        Coord {
            x: self.start_pos.x + self.axes_r.x * move_dist,
            y: self.start_pos.y + self.axes_r.y * move_dist,
            z: self.start_pos.z + self.axes_r.z * move_dist,
        }
    }
}

/// A move and its place in a trapq move list (list_prev_entry /
/// list_next_entry). A move outside any list (a stack `struct move` in C)
/// has no neighbours; prev() / next() then return the move itself.
#[derive(Clone, Copy)]
pub struct MoveRef<'a> {
    list: Option<&'a VecDeque<Move>>,
    idx: usize,
    pub m: &'a Move,
}

impl<'a> MoveRef<'a> {
    pub fn new(list: &'a VecDeque<Move>, idx: usize) -> Self {
        MoveRef { list: Some(list), idx, m: &list[idx] }
    }

    pub fn standalone(m: &'a Move) -> Self {
        MoveRef { list: None, idx: 0, m }
    }

    pub fn prev(&self) -> MoveRef<'a> {
        match self.list {
            Some(l) if self.idx > 0 => MoveRef::new(l, self.idx - 1),
            _ => *self,
        }
    }

    pub fn next(&self) -> MoveRef<'a> {
        match self.list {
            Some(l) if self.idx + 1 < l.len() => MoveRef::new(l, self.idx + 1),
            _ => *self,
        }
    }

    pub fn index(&self) -> usize {
        self.idx
    }
}

impl std::ops::Deref for MoveRef<'_> {
    type Target = Move;
    fn deref(&self) -> &Move {
        self.m
    }
}

pub const NEVER_TIME: f64 = 9999999999999999.9;

#[derive(Debug)]
pub struct TrapQ {
    pub moves: VecDeque<Move>,
    pub history: VecDeque<Move>,
}

pub type TrapQRef = Rc<RefCell<TrapQ>>;

/// struct pull_move
#[derive(Clone, Copy, Debug, Default)]
pub struct PullMove {
    pub print_time: f64,
    pub move_t: f64,
    pub start_v: f64,
    pub accel: f64,
    pub start_x: f64,
    pub start_y: f64,
    pub start_z: f64,
    pub x_r: f64,
    pub y_r: f64,
    pub z_r: f64,
}

const MAX_NULL_MOVE: f64 = 1.0;

impl Default for TrapQ {
    fn default() -> Self {
        Self::new()
    }
}

impl TrapQ {
    // Allocate a new 'trapq' object
    pub fn new() -> Self {
        let head_sentinel = Move { print_time: -1.0, ..Move::default() };
        let tail_sentinel = Move { print_time: NEVER_TIME, move_t: NEVER_TIME, ..Move::default() };
        TrapQ { moves: VecDeque::from([head_sentinel, tail_sentinel]), history: VecDeque::new() }
    }

    pub fn alloc() -> TrapQRef {
        Rc::new(RefCell::new(Self::new()))
    }

    fn tail(&self) -> usize {
        self.moves.len() - 1
    }

    // Update the list sentinels
    pub fn check_sentinels(&mut self) {
        let tail = self.tail();
        if self.moves[tail].print_time != 0. {
            // Already up to date
            return;
        }
        if tail - 1 == 0 {
            // No moves at all on this list
            self.moves[tail].print_time = NEVER_TIME;
            return;
        }
        let m = self.moves[tail - 1];
        self.moves[tail].print_time = m.print_time + m.move_t;
        self.moves[tail].start_pos = m.get_coord(m.move_t);
    }

    // Add a move to the trapezoid velocity queue
    pub fn add_move(&mut self, m: Move) {
        let tail = self.tail();
        let prev = self.moves[tail - 1];
        if prev.print_time + prev.move_t < m.print_time {
            // Add a null move to fill time gap
            let mut null_move = Move { start_pos: m.start_pos, ..Move::default() };
            if prev.print_time == 0. && m.print_time > MAX_NULL_MOVE {
                // Limit the first null move to improve numerical stability
                null_move.print_time = m.print_time - MAX_NULL_MOVE;
            } else {
                null_move.print_time = prev.print_time + prev.move_t;
            }
            null_move.move_t = m.print_time - null_move.print_time;
            self.moves.insert(self.tail(), null_move);
        }
        self.moves.insert(self.tail(), m);
        let tail = self.tail();
        self.moves[tail].print_time = 0.;
    }

    // Fill and add a move to the trapezoid velocity queue
    #[allow(clippy::too_many_arguments)]
    pub fn append(
        &mut self,
        mut print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos_x: f64,
        start_pos_y: f64,
        start_pos_z: f64,
        axes_r_x: f64,
        axes_r_y: f64,
        axes_r_z: f64,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    ) {
        let mut start_pos = Coord { x: start_pos_x, y: start_pos_y, z: start_pos_z };
        let axes_r = Coord { x: axes_r_x, y: axes_r_y, z: axes_r_z };
        if accel_t != 0. {
            let m = Move { print_time, move_t: accel_t, start_v, half_accel: 0.5 * accel, start_pos, axes_r };
            self.add_move(m);
            print_time += accel_t;
            start_pos = m.get_coord(accel_t);
        }
        if cruise_t != 0. {
            let m = Move { print_time, move_t: cruise_t, start_v: cruise_v, half_accel: 0., start_pos, axes_r };
            self.add_move(m);
            print_time += cruise_t;
            start_pos = m.get_coord(cruise_t);
        }
        if decel_t != 0. {
            let m = Move { print_time, move_t: decel_t, start_v: cruise_v, half_accel: -0.5 * accel, start_pos, axes_r };
            self.add_move(m);
        }
    }

    // Expire any moves older than `print_time` from the trapezoid velocity queue
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        // Move expired moves from main "moves" list to "history" list
        loop {
            if self.moves.len() == 2 {
                let tail = self.tail();
                self.moves[tail].print_time = NEVER_TIME;
                break;
            }
            let m = self.moves[1];
            if m.print_time + m.move_t > print_time {
                break;
            }
            self.moves.remove(1);
            if m.start_v != 0. || m.half_accel != 0. {
                self.history.push_front(m);
            }
        }
        // Free old moves from history list
        while self.history.len() > 1 {
            let m = self.history[self.history.len() - 1];
            if m.print_time + m.move_t > clear_history_time {
                break;
            }
            self.history.pop_back();
        }
    }

    // Note a position change in the trapq history
    pub fn set_position(&mut self, print_time: f64, pos_x: f64, pos_y: f64, pos_z: f64) {
        // Flush all moves from trapq
        self.finalize_moves(NEVER_TIME, 0.);

        // Prune any moves in the trapq history that were interrupted
        while let Some(m) = self.history.front_mut() {
            if m.print_time < print_time {
                if m.print_time + m.move_t > print_time {
                    m.move_t = print_time - m.print_time;
                }
                break;
            }
            self.history.pop_front();
        }

        // Add a marker to the trapq history
        let m = Move { print_time, start_pos: Coord { x: pos_x, y: pos_y, z: pos_z }, ..Move::default() };
        self.history.push_front(m);
    }

    // Return history of movement queue
    pub fn extract_old(&self, max: usize, start_time: f64, end_time: f64) -> Vec<PullMove> {
        let mut res = vec![];
        for m in &self.history {
            if start_time >= m.print_time + m.move_t || res.len() >= max {
                break;
            }
            if end_time <= m.print_time {
                continue;
            }
            res.push(PullMove {
                print_time: m.print_time,
                move_t: m.move_t,
                start_v: m.start_v,
                accel: 2. * m.half_accel,
                start_x: m.start_pos.x,
                start_y: m.start_pos.y,
                start_z: m.start_pos.z,
                x_r: m.axes_r.x,
                y_r: m.axes_r.y,
                z_r: m.axes_r.z,
            });
        }
        res
    }
}
