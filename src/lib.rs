// Kaliru - Kalico (Klipper fork) ported to Rust

/// The typed downcasts of a trait object whose trait has `Any` as a
/// supertrait: `obj.downcast_ref::<T>()`, `obj.downcast_mut::<T>()`,
/// `obj.is::<T>()` (trait upcasting to `dyn Any`; no `as_any` per type).
/// Invoke it next to the trait: `crate::impl_downcast!(dyn Trait);`
#[macro_export]
macro_rules! impl_downcast {
    ($t:ty) => {
        impl $t {
            #[inline]
            pub fn downcast_ref<T: std::any::Any>(&self) -> Option<&T> {
                (self as &dyn std::any::Any).downcast_ref::<T>()
            }
            #[inline]
            pub fn downcast_mut<T: std::any::Any>(&mut self) -> Option<&mut T> {
                (self as &mut dyn std::any::Any).downcast_mut::<T>()
            }
            #[inline]
            pub fn is<T: std::any::Any>(&self) -> bool {
                (self as &dyn std::any::Any).is::<T>()
            }
        }
    };
}
pub mod chelper;
pub mod clocksync;
pub mod configfile;
pub mod configparser;
pub mod error;
pub mod events;
pub mod extras;
pub mod gcode;
pub mod greenlet;
pub mod jinja;
pub mod kinematics;
pub mod logging;
pub mod mathutil;
pub mod numpy_fft;
pub mod scipy_signal;
pub mod mcu;
pub mod msgproto;
pub mod pins;
#[cfg(feature = "plugins")]
pub mod plugins;
pub mod printer;
pub mod pyfmt;
pub mod pyformat;
pub mod pyliteral;
#[cfg(feature = "python")]
pub mod pysyntax;
pub mod pyglob;
pub mod pyval;
pub mod queuelogger;
pub mod reactor;
pub mod running;
pub mod serialhdl;
pub mod stepper;
pub mod toolhead;
pub mod util;
pub mod webhooks;
