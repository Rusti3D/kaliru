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
        }
    )*};
}
modify!(u8, u16, u32);
