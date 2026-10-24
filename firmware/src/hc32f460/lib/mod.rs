// The lib/hc32f460 (Huada driver library) functions Kalico's hc32f460 code
// calls, one file per C file. (Its DDL_ASSERT parameter checks are compiled
// out in Kalico's build; the en_result_t results Kalico does not use are
// left out.)
#![allow(non_snake_case, non_upper_case_globals, non_camel_case_types)]
pub mod hc32f460_adc;
pub mod hc32f460_gpio;
pub mod hc32f460_pwc;
pub mod hc32f460_timera;
pub mod hc32f460_usart;
pub mod system_hc32f460;
