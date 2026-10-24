// A memory mapped register: the CMSIS __IO / __I / __O fields (pico-sdk io_rw_32
// ...), accessed with volatile reads and writes as the C struct member accesses are.
use core::cell::UnsafeCell;

#[repr(transparent)]
pub struct Reg<T: Copy>(UnsafeCell<T>);

impl<T: Copy> Reg<T> {
    // (a C volatile variable)
    pub const fn new(v: T) -> Self {
        Reg(UnsafeCell::new(v))
    }
    #[inline(always)]
    pub fn read(&self) -> T {
        unsafe { core::ptr::read_volatile(self.0.get()) }
    }
    #[inline(always)]
    pub fn write(&self, v: T) {
        unsafe { core::ptr::write_volatile(self.0.get(), v) }
    }
    #[inline(always)]
    pub fn ptr(&self) -> *mut T {
        self.0.get()
    }
}

// A bit field of a register: the C `REG_f.NAME` of the Huada (hc32f460)
// headers, positions from gcc (tools/gen_stm32_regs.py --hc32)
#[derive(Clone, Copy)]
pub struct Field {
    pub pos: u32,
    pub width: u32,
}

impl Field {
    // the field's bits in its register
    pub const fn mask(self) -> u32 {
        (u32::MAX >> (32 - self.width)) << self.pos
    }
    // the field of a register value
    pub const fn get(self, word: u32) -> u32 {
        (word & self.mask()) >> self.pos
    }
    // a register value with the field set (C: an assignment to the
    // field of a struct value)
    pub const fn set(self, word: u32, v: u32) -> u32 {
        (word & !self.mask()) | ((v << self.pos) & self.mask())
    }
}

macro_rules! modify {
    ($($t:ty),*) => {$(
        impl Reg<$t> {
            // x |= v
            #[inline(always)]
            pub fn or(&self, v: $t) {
                self.write(self.read() | v)
            }
            // x &= v
            #[inline(always)]
            pub fn and(&self, v: $t) {
                self.write(self.read() & v)
            }
            // x ^= v
            #[inline(always)]
            pub fn xor(&self, v: $t) {
                self.write(self.read() ^ v)
            }
            // MODIFY_REG(x, clear, set)
            #[inline(always)]
            pub fn modify(&self, clear: $t, set: $t) {
                self.write((self.read() & !clear) | set)
            }
            // x_f.FIELD
            #[inline(always)]
            pub fn field(&self, f: Field) -> u32 {
                f.get(self.read() as u32)
            }
            // x_f.FIELD = v (read, modify, write of the register)
            #[inline(always)]
            pub fn set_field(&self, f: Field, v: u32) {
                self.write(f.set(self.read() as u32, v) as $t)
            }
        }
    )*};
}
modify!(u8, u16, u32);
