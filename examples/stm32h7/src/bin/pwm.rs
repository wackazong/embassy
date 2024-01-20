#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::{self, PB5};
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::{interrupt, Config};
use {defmt_rtt as _, panic_probe as _};

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn UART4() {
    EXECUTOR_HIGH.on_interrupt()
}

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL60,
            divp: Some(PllDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 480 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 240 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale0;
        config.rcc.supply_config = SupplyConfig::DirectSMPS;
    }
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    let ch1 = PwmPin::new_ch1(p.PA6, OutputType::PushPull);
    let mut pwm = SimplePwm::new(p.TIM3, Some(ch1), None, None, None, khz(10), Default::default());
    let max = pwm.get_max_duty();
    pwm.enable(Channel::Ch1);

    info!("PWM initialized");
    info!("PWM max duty {}", max);
    pwm.set_duty(Channel::Ch1, max / 2);

    // High-priority executor: UART4, priority level 6
    interrupt::UART4.set_priority(Priority::P0);
    let spawner = EXECUTOR_HIGH.start(interrupt::UART4);
    unwrap!(spawner.spawn(task(p.PB5, p.EXTI5, p.PA5)));

    let mut cpuid: cortex_m::peripheral::CPUID = unsafe { core::mem::transmute(()) };
    let mut scb: cortex_m::peripheral::SCB = unsafe { core::mem::transmute(()) };
    scb.enable_dcache(&mut cpuid);
    scb.enable_icache();
    //scb.set_sleepdeep();
    scb.set_sleeponexit();

    loop {
        cortex_m::asm::wfi();
    }
}

#[embassy_executor::task]
async fn task(pb5: peripherals::PB5, exti5: peripherals::EXTI5, pa5: peripherals::PA5) {
    let mut exti = ExtiInput::new(Input::new(pb5, Pull::None), exti5);
    let mut out = Output::new(pa5, Level::Low, Speed::VeryHigh);
    loop {
        exti.wait_for_rising_edge().await;
        out.set_high();
        cortex_m::asm::delay(50);
        out.set_low();
    }
}
