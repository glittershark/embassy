//! Low-power support.
//!
//! The STM32 line of microcontrollers support various deep-sleep modes which exploit clock-gating
//! to reduce power consumption. The `embassy-stm32` HAL provides a `sleep()` function which
//! can use knowledge of which peripherals are currently blocked upon to transparently and safely
//! enter such low-power modes including `STOP1` and `STOP2` when possible.
//!
//! `sleep()` determines which peripherals are active by their RCC state; consequently,
//! low-power states can only be entered if peripherals which block stop have been `drop`'d and if
//! peripherals that do not block stop are busy. Peripherals which never block stop include:
//!
//!  * `GPIO`
//!  * `RTC`
//!
//! Other peripherals which block stop when busy include (this list may be stale):
//!
//!  * `I2C`
//!  * `USART`
//!
//! Since entering and leaving low-power modes typically incurs a significant latency, `sleep()`
//! will only attempt to enter when the next timer event is at least [`config.min_stop_pause`] in the future.
//!
//! `embassy-stm32` also provides an `embassy-executor` platform implementation that integrates `sleep()` into the main loop. It is available
//! in the `embassy_stm32::executor` module, and is enabled by the `executor-thread` or `executor-interrupt` features. This stm32-specific
//! executor is the preferred way to lower power consumption if you're using `async`, instead of calling `sleep()` directly.

use core::mem;
use core::sync::atomic::{Ordering, compiler_fence};

use cortex_m::peripheral::SCB;
use critical_section::CriticalSection;

#[cfg(not(feature = "_lp-time-driver"))]
use crate::interrupt;
pub use crate::rcc::StopMode;
use crate::rcc::get_stop_mode;
use crate::time_driver::{LPTimeDriver, get_driver};

#[cfg(not(any(stm32u0, feature = "_lp-time-driver")))]
foreach_interrupt! {
    (RTC, rtc, $block:ident, WKUP, $irq:ident) => {
        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $irq() {
        }
    };
}

#[cfg(stm32u0)]
foreach_interrupt! {
    (RTC, rtc, $block:ident, TAMP, $irq:ident) => {
        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $irq() {
        }
    };
}

#[cfg(any(stm32l4, stm32l5, stm32u5, stm32u3, stm32wba, stm32wb, stm32wl, stm32u0))]
use crate::pac::pwr::vals::Lpms;

#[cfg(any(stm32l4, stm32l5, stm32u5, stm32u3, stm32wba, stm32wb, stm32wl, stm32u0))]
impl Into<Lpms> for StopMode {
    fn into(self) -> Lpms {
        match self {
            StopMode::Stop1 => Lpms::STOP1,
            #[cfg(not(stm32wba))]
            StopMode::Standby | StopMode::Stop2 => Lpms::STOP2,
            #[cfg(stm32wba)]
            // WBA STOP2 is auto-entered by hardware when LPMS=STOP0 and
            // the 2.4 GHz radio is in deep sleep. It's not a separate LPMS value.
            StopMode::Standby | StopMode::Stop2 => Lpms::STOP0,
        }
    }
}

mod platform {
    use critical_section::CriticalSection;

    use crate::rcc::StopMode;

    /// The clocks that stop mode changes, as they were before `enter_stop`.
    #[cfg(stm32wb)]
    #[derive(Clone, Copy)]
    struct ClocksBeforeStop {
        cr: crate::pac::rcc::regs::Cr,
        sw: crate::pac::rcc::vals::Sw,
        smpssel: crate::pac::rcc::vals::Smps,
    }

    #[cfg(stm32wb)]
    static mut CLOCKS_BEFORE_STOP: Option<ClocksBeforeStop> = None;

    // This is `ExitLowPower` of ST's applications, which only turns HSE on again. Stop mode
    // also turns the PLLs and MSI off, so all of them are restored here.
    #[cfg(stm32wb)]
    fn exit_stop_stm32wb(_cs: CriticalSection<'_>) {
        use crate::hsem::get_hsem;
        use crate::pac::RCC;

        // Held since `enter_stop` if CPU2 was running then.
        get_hsem(4).unlock(0);

        // SAFETY: only this function and `enter_stop` use the static, both in a critical
        // section.
        let Some(before) = (unsafe { CLOCKS_BEFORE_STOP }) else {
            return;
        };
        unsafe { CLOCKS_BEFORE_STOP = None };

        let sem3_mutex = get_hsem(3).blocking_lock(0);

        // Each of these is still on if the system did not stop, and CPU2 may have turned
        // HSE on already. HSI is left on: CPU2 may be using it.
        if before.cr.msion() {
            RCC.cr().modify(|w| w.set_msion(true));
            while !RCC.cr().read().msirdy() {}
        }
        if before.cr.hseon() {
            RCC.cr().modify(|w| w.set_hseon(true));
            while !RCC.cr().read().hserdy() {}
        }
        if before.cr.pllon() {
            RCC.cr().modify(|w| w.set_pllon(true));
            while !RCC.cr().read().pllrdy() {}
        }
        if before.cr.pllsai1on() {
            RCC.cr().modify(|w| w.set_pllsai1on(true));
            while !RCC.cr().read().pllsai1rdy() {}
        }

        if RCC.cfgr().read().sws() != before.sw {
            RCC.cfgr().modify(|w| w.set_sw(before.sw));
            while RCC.cfgr().read().sws() != before.sw {}
        }
        if RCC.smpscr().read().smpssel() != before.smpssel {
            RCC.smpscr().modify(|w| w.set_smpssel(before.smpssel));
        }

        drop(sem3_mutex);
    }

    /// Enter stop mode
    pub fn enter_stop(_cs: CriticalSection, stop_mode: StopMode) -> Result<(), ()> {
        // This is `EnterLowPower` of ST's applications (`stm32_lpm_if.c`). CPU2 follows the
        // same protocol.
        #[cfg(stm32wb)]
        fn enter_stop_stm32wb(
            _cs: CriticalSection<'_>,
        ) -> crate::hsem::HardwareSemaphoreMutex<'static, crate::peripherals::HSEM> {
            use crate::hsem::get_hsem;
            use crate::pac::rcc::vals::{Smps, Sw};
            use crate::pac::{PWR, RCC};

            // The system clock after stop is HSI16.
            RCC.cfgr().modify(|w| w.set_stopwuck(true));

            // CPU2 only holds the RCC semaphore while it changes the clocks.
            let sem3_mutex = get_hsem(3).blocking_lock(0);
            trace!("low power: got sem3");

            // SAFETY: only this function and `exit_stop` use the static, both in a critical
            // section.
            unsafe {
                CLOCKS_BEFORE_STOP = Some(ClocksBeforeStop {
                    cr: RCC.cr().read(),
                    sw: RCC.cfgr().read().sw(),
                    smpssel: RCC.smpscr().read().smpssel(),
                });
            }

            let extscr = PWR.extscr().read();
            let cpu2_stopped = extscr.c2ds() || extscr.c2sbf();
            match get_hsem(4).try_lock(0) {
                // CPU2 is running, and needs the clocks as they are. `exit_stop` releases the
                // semaphore. Until then it tells CPU2 that this core has stopped, so that CPU2
                // switches to HSI itself before it stops.
                Some(sem4_mutex) if !cpu2_stopped => {
                    trace!("low power: got sem4");
                    core::mem::forget(sem4_mutex);
                    return sem3_mutex;
                }
                // CPU2 has stopped, or it has the semaphore because it is about to. The
                // semaphore is released here if it was granted.
                _ => {}
            }

            // The system stops with HSI16 as its clock, as in ST's sequence.
            RCC.cr().modify(|w| {
                w.set_hsion(true);
            });
            while !RCC.cr().read().hsirdy() {}

            RCC.cfgr().modify(|w| {
                w.set_sw(Sw::HSI);
            });
            RCC.smpscr().modify(|w| {
                w.set_smpssel(Smps::HSI);
            });
            while !RCC.cfgr().read().sws().eq(&Sw::HSI) {}

            sem3_mutex
        }

        // The SMPS stays as the application set it. The chip opens it by itself in Stop 1 and
        // Stop 2 and closes it again afterwards, and ST's sequence does not touch it. Turning
        // it off here left it off for good after the first stop.
        #[cfg(stm32wb)]
        let mutex = enter_stop_stm32wb(_cs);

        #[cfg(any(stm32l4, stm32l5, stm32u5, stm32u3, stm32u0, stm32wb, stm32wba, stm32wl))]
        {
            #[cfg(not(feature = "_core-cm0p"))]
            crate::pac::PWR.cr1().modify(|m| m.set_lpms(stop_mode.into()));
            #[cfg(feature = "_core-cm0p")]
            crate::pac::PWR.c2cr1().modify(|m| m.set_lpms(stop_mode.into()));
        }
        #[cfg(stm32h5)]
        crate::pac::PWR.pmcr().modify(|v| {
            use crate::pac::pwr::vals;
            v.set_lpms(vals::Lpms::STOP);
            v.set_svos(vals::Svos::SCALE3);
        });

        #[cfg(stm32l0)]
        {
            use crate::pac::pwr::vals::Pdds;
            crate::pac::PWR.cr().modify(|w| {
                w.set_pdds(Pdds::STOP_MODE);
                w.set_cwuf(true);
            });
        }

        #[cfg(stm32wb)]
        drop(mutex);

        let _ = stop_mode;

        Ok(())
    }

    /// Clear any previous stop flags
    pub fn clear_flags() {
        #[cfg(stm32wl)]
        crate::pac::PWR.extscr().modify(|w| {
            #[cfg(not(feature = "_core-cm0p"))]
            w.set_c1cssf(true);
            #[cfg(feature = "_core-cm0p")]
            w.set_c2cssf(true);
        });
        #[cfg(stm32wba)]
        crate::pac::PWR.sr().modify(|w| w.set_cssf(true));
        #[cfg(stm32wb)]
        crate::pac::PWR.extscr().modify(|w| w.set_c1cssf(true));

        #[cfg(stm32l0)]
        crate::pac::PWR.cr().modify(|w| w.set_cwuf(true));
    }

    /// Exit stop mode, reinitializing timer and rcc if required
    pub fn exit_stop(_cs: CriticalSection) {
        #[cfg(any(stm32l0, stm32wl, stm32wb, stm32wba))]
        {
            // stm32wl5x is dual core and we don't want BOTH cores to re-initialize RCC so we hold a lock
            #[cfg(stm32wl5x)]
            let lock = crate::hsem::get_hsem(3).blocking_lock(0);

            #[cfg(any(stm32wl, stm32wb))]
            let es = crate::pac::PWR.extscr().read();

            #[cfg(stm32wba)]
            let es = crate::pac::PWR.sr().read();

            #[cfg(stm32l0)]
            let es = crate::pac::PWR.csr().read();

            // we need to re-initialize RCC if *BOTH* cores have been in some STOP mode!
            #[cfg(any(stm32l0, stm32wl, stm32wba))]
            let re_initialize_rcc = {
                #[cfg(stm32wl5x)]
                {
                    // core 1 in any STOP mode AND core 2 in any STOP mode
                    (es.c1stopf() || es.c1stop2f()) && (es.c2stopf() || es.c2stop2f())
                }
                #[cfg(stm32wlex)]
                {
                    es.c1stop2f() || es.c1stopf()
                }
                #[cfg(stm32wba)]
                {
                    es.stopf()
                }
                #[cfg(stm32l0)]
                {
                    es.wuf()
                }
            };

            #[cfg(any(stm32wl, stm32wba))]
            let re_initialize_timer = {
                #[cfg(all(stm32wl, not(feature = "_core-cm0p")))]
                {
                    es.c1stop2f()
                }
                #[cfg(all(stm32wl, feature = "_core-cm0p"))]
                {
                    es.c2stop2f()
                }
                #[cfg(stm32wba)]
                {
                    es.stopf()
                }
            };

            #[cfg(any(stm32l0, stm32wl, stm32wba))]
            if re_initialize_rcc {
                // when we wake from any stop mode we need to re-initialize the rcc
                crate::rcc::reinit_saved(_cs);
            }

            // On stm32wb the clocks are restored without the detour through MSI that
            // `reinit_saved` makes: CPU2 is clocked from the system clock too, and may be using
            // the radio.
            #[cfg(stm32wb)]
            exit_stop_stm32wb(_cs);

            #[cfg(stm32wba)]
            match (es.stopf(), es.stop2f()) {
                (true, true) => debug!("low power: WBA woke from STOP2"),
                (true, false) => debug!("low power: WBA woke from STOP0/1"),
                _ => {}
            };

            #[cfg(stm32wl)]
            match (es.c1stopf(), es.c1stop2f()) {
                (true, false) => debug!("low power: cpu1 has been in STOP1"),
                (false, true) => debug!("low power: cpu1 has been in STOP2"),
                (true, true) => debug!("low power: cpu1 has been in STOP1 and STOP2 ???"),
                (false, false) => trace!("low power: cpu1 stop mode not entered"),
            };

            #[cfg(stm32wl5x)]
            // TODO: only for the current cpu
            match (es.c2stopf(), es.c2stop2f()) {
                (true, false) => debug!("low power: cpu2 has been in STOP1"),
                (false, true) => debug!("low power: cpu2 has been in STOP2"),
                (true, true) => debug!("low power: cpu2 has been in STOP1 and STOP2 ???"),
                (false, false) => trace!("low power: cpu2 stop mode not entered"),
            };

            #[cfg(stm32wb)]
            match (es.c1stopf(), es.c2stopf()) {
                (true, false) => debug!("low power: cpu1 has been in STOP"),
                (false, true) => debug!("low power: cpu2 has been in STOP"),
                (true, true) => debug!("low power: cpu1 and cpu2 have been in STOP"),
                (false, false) => trace!("low power: stop mode not entered"),
            };

            #[cfg(stm32l0)]
            match es.wuf() {
                true => debug!("low power: L0 has been in stop"),
                _ => {}
            };

            clear_flags();

            #[cfg(stm32wl5x)]
            drop(lock);

            #[cfg(any(stm32wl, stm32wba))]
            if re_initialize_timer {
                trace!("low power: re-initializing timer");
                // when we wake from STOP2, we need to re-initialize the time driver
                #[cfg(not(feature = "_lp-time-driver"))]
                super::get_driver().init_timer(_cs);
            }
        }
    }
}

unsafe fn on_wakeup_irq_or_event() {
    if !get_driver().is_stopped() {
        //trace!("low power: time driver not stopped!");
        return;
    }

    critical_section::with(|cs| {
        platform::exit_stop(cs);

        get_driver().resume_time(cs);
        trace!("low power: resumed");
    });
}

fn configure_pwr(cs: CriticalSection) {
    const fn get_scb() -> SCB {
        unsafe { mem::transmute(()) }
    }

    get_scb().clear_sleepdeep();
    platform::clear_flags();

    compiler_fence(Ordering::Acquire);

    let Some(stop_mode) = get_stop_mode(cs) else {
        //trace!("low power: no stop mode available");
        return;
    };

    // The next alarm is too close: sleep without stopping, so that the time
    // driver keeps running and wakes the core for it.
    if get_driver().pause_time(cs).is_err() {
        trace!("low power: not entering stop: the next alarm is too close");
        return;
    }

    // Time is paused now, and the RTC wakes the core for the next alarm.
    // Sleep without stopping; `on_wakeup_irq_or_event` resumes time.
    if platform::enter_stop(cs, stop_mode).is_err() {
        trace!("low power: not entering stop");
        return;
    }

    #[cfg(stm32l0)]
    trace!("low power: enter stop");
    #[cfg(not(stm32l0))]
    trace!("low power: enter stop: {}", stop_mode);

    #[cfg(not(feature = "low-power-debug-with-sleep"))]
    get_scb().set_sleepdeep();
}

/// Sleep with WFI, attempting to enter the deepest STOP mode possible.
///
/// If it's not possible to enter any STOP mode due to running peripherals it will
/// still do a `WFI` sleep. Therefore this function is equivalent to `WFI` except
/// with lower power consumption and higher latency.
///
/// ## SAFETY
///
/// Care must be taken that we have ensured that the system is ready to go to deep
/// sleep, otherwise HAL peripherals may misbehave. HAL drivers automatically prevent
/// sleep as needed, but you might have to do it manually if you're using some peripherals
/// with the PAC directly.
pub unsafe fn sleep(cs: CriticalSection) {
    configure_pwr(cs);

    #[cfg(feature = "low-power-defmt-flush")]
    defmt::flush();

    cortex_m::asm::dsb();
    cortex_m::asm::wfi();

    on_wakeup_irq_or_event();
}
