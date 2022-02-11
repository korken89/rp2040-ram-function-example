//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::convert::TryInto;

use cortex_m_rt::{entry, exception, ExceptionFrame};
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

use bsp::hal::pac;
use bsp::hal::rom_data;

static mut BOOT2_COPYOUT: [u32; 64] = [0; 64];

unsafe fn boot_2_copyout() {
    let xip_base = 0x10000000 as *const u32;
    rom_data::memcpy(BOOT2_COPYOUT.as_mut_ptr() as _, xip_base as _, 256);
}

#[link_section = ".data"]
unsafe fn set_cs(level: bool) {
    (&*pac::IO_QSPI::ptr())
        .gpio_qspiss
        .gpio_ctrl
        .modify(|_, w| {
            if level {
                w.outover().high()
            } else {
                w.outover().low()
            }
        });
}

#[link_section = ".data"]
#[inline(never)]
unsafe fn do_flash_cmd(txbuf: *const u8, rxbuf: *mut u8, count: usize) {
    // Load important addresses to the stack
    let connect_internal_flash = rom_data::connect_internal_flash;
    let flash_exit_xip = rom_data::flash_exit_xip;
    let flash_flush_cache = rom_data::flash_flush_cache;

    boot_2_copyout();

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    connect_internal_flash();
    flash_exit_xip();

    set_cs(false);

    let ssi = &*pac::XIP_SSI::ptr();

    for i in 0..count {
        while !ssi.sr.read().tfnf().bit_is_set() {}
        ssi.dr0.write(|w| w.dr().bits(*txbuf.add(i) as _));

        while !ssi.sr.read().rfne().bit_is_set() {}
        core::ptr::write(rxbuf.add(i), ssi.dr0.read().dr().bits() as _);
    }

    set_cs(true);

    flash_flush_cache();
    flash_enable_xip_via_boot2();
}

#[link_section = ".data"]
unsafe fn flash_enable_xip_via_boot2() {
    let ptr = (BOOT2_COPYOUT.as_mut_ptr() as *const u8).add(1) as *const ();
    let start: extern "C" fn() = core::mem::transmute(ptr);
    start();
}

fn read_uid() -> u64 {
    const FLASH_RUID_CMD: u8 = 0x4b;
    const FLASH_RUID_DUMMY_BYTES: usize = 4;
    const FLASH_RUID_DATA_BYTES: usize = 8;
    const FLASH_RUID_TOTAL_BYTES: usize = 1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES;

    let mut txbuf = [0; FLASH_RUID_TOTAL_BYTES];
    let mut rxbuf = [0; FLASH_RUID_TOTAL_BYTES];

    txbuf[0] = FLASH_RUID_CMD;

    cortex_m::interrupt::free(|_| unsafe {
        do_flash_cmd(txbuf.as_ptr(), rxbuf.as_mut_ptr(), FLASH_RUID_TOTAL_BYTES)
    });

    u64::from_le_bytes(rxbuf[FLASH_RUID_DUMMY_BYTES + 1..].try_into().unwrap())
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let uid = read_uid();
    info!("uid: {:#018x}", uid);

    #[allow(clippy::empty_loop)]
    loop {}
}

#[exception]
unsafe fn HardFault(eh: &ExceptionFrame) -> ! {
    panic!("Hardfault: {:?}", eh);
}
