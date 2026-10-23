// CMSIS Cortex-M0 Device Peripheral Access Layer System Source File
// (lib/stm32f0/system_stm32f0xx.c): the SystemInit() the firmware calls.
//
// Copyright (c) 2016 STMicroelectronics. All rights reserved.
// (BSD-3-Clause, see Kalico lib/stm32f0)
#![allow(non_snake_case)]
use super::internal::RCC;

// Setup the microcontroller system: reset the RCC clock configuration to
// the default reset state (the chip variant is the -D define of the build)
pub fn SystemInit() {
    let rcc = unsafe { &*RCC };
    // Set HSION bit
    rcc.CR.or(0x00000001);

    #[cfg(any(has_STM32F051x8, has_STM32F058x8))]
    {
        // Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE and MCOSEL[2:0] bits
        rcc.CFGR.and(0xF8FFB80C);
    }
    #[cfg(not(any(has_STM32F051x8, has_STM32F058x8)))]
    {
        // Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE, MCOSEL[2:0], MCOPRE[2:0] and PLLNODIV bits
        rcc.CFGR.and(0x08FFB80C);
    }

    // Reset HSEON, CSSON and PLLON bits
    rcc.CR.and(0xFEF6FFFF);

    // Reset HSEBYP bit
    rcc.CR.and(0xFFFBFFFF);

    // Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits
    rcc.CFGR.and(0xFFC0FFFF);

    // Reset PREDIV[3:0] bits
    rcc.CFGR2.and(0xFFFFFFF0);

    #[cfg(any(has_STM32F072xB, has_STM32F078xx))]
    {
        // Reset USART2SW[1:0], USART1SW[1:0], I2C1SW, CECSW, USBSW and ADCSW bits
        rcc.CFGR3.and(0xFFFCFE2C);
    }
    #[cfg(has_STM32F071xB)]
    {
        // Reset USART2SW[1:0], USART1SW[1:0], I2C1SW, CECSW and ADCSW bits
        rcc.CFGR3.and(0xFFFFCEAC);
    }
    #[cfg(any(has_STM32F091xC, has_STM32F098xx))]
    {
        // Reset USART3SW[1:0], USART2SW[1:0], USART1SW[1:0], I2C1SW, CECSW and ADCSW bits
        rcc.CFGR3.and(0xFFF0FEAC);
    }
    #[cfg(any(has_STM32F030x6, has_STM32F030x8, has_STM32F031x6, has_STM32F038xx, has_STM32F030xC))]
    {
        // Reset USART1SW[1:0], I2C1SW and ADCSW bits
        rcc.CFGR3.and(0xFFFFFEEC);
    }
    #[cfg(any(has_STM32F051x8, has_STM32F058xx))]
    {
        // Reset USART1SW[1:0], I2C1SW, CECSW and ADCSW bits
        rcc.CFGR3.and(0xFFFFFEAC);
    }
    #[cfg(any(has_STM32F042x6, has_STM32F048xx))]
    {
        // Reset USART1SW[1:0], I2C1SW, CECSW, USBSW and ADCSW bits
        rcc.CFGR3.and(0xFFFFFE2C);
    }
    #[cfg(any(has_STM32F070x6, has_STM32F070xB))]
    {
        // Reset USART1SW[1:0], I2C1SW, USBSW and ADCSW bits
        rcc.CFGR3.and(0xFFFFFE6C);
        // Set default USB clock to PLLCLK, since there is no HSI48
        rcc.CFGR3.or(0x00000080);
    }

    // Reset HSI14 bit
    rcc.CR2.and(0xFFFFFFFE);

    // Disable all interrupts
    rcc.CIR.write(0x00000000);
}
