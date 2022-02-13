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

#[inline(always)]
#[link_section = ".data.ram_func"]
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

#[no_mangle]
#[link_section = ".data.ram_func"]
#[inline(never)]
pub extern "C" fn do_flash_cmd2(txrxbuf: *mut u8, len: u32) {
    unsafe {
        let txrxbuf = core::slice::from_raw_parts_mut(txrxbuf, len as _);
        // Load important addresses to the stack
        let connect_internal_flash = rom_data::connect_internal_flash;
        let flash_exit_xip = rom_data::flash_exit_xip;
        let flash_flush_cache = rom_data::flash_flush_cache;

        let mut boot2: core::mem::MaybeUninit<[u8; 256]> = core::mem::MaybeUninit::uninit();

        let xip_base = 0x10000000 as *const u32;
        rom_data::memcpy(boot2.as_mut_ptr() as _, xip_base as _, 256);

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        connect_internal_flash();
        flash_exit_xip();

        set_cs(false);

        let ssi = &*pac::XIP_SSI::ptr();

        for b in txrxbuf {
            while !ssi.sr.read().tfnf().bit_is_set() {}
            ssi.dr0.write(|w| w.dr().bits(*b as _));

            while !ssi.sr.read().rfne().bit_is_set() {}
            *b = ssi.dr0.read().dr().bits() as _;
        }

        set_cs(true);

        flash_flush_cache();

        let ptr = (boot2.as_mut_ptr() as *const u8).add(1) as *const ();
        let start: extern "C" fn() = core::mem::transmute(ptr);
        start();
    }
}

#[link_section = ".data.ram_func"]
#[inline(never)]
unsafe fn do_flash_cmd(txrxbuf: &mut [u8]) {
    // Load important addresses to the stack
    let connect_internal_flash = rom_data::connect_internal_flash;
    let flash_exit_xip = rom_data::flash_exit_xip;
    let flash_flush_cache = rom_data::flash_flush_cache;

    let mut boot2: core::mem::MaybeUninit<[u8; 256]> = core::mem::MaybeUninit::uninit();

    let xip_base = 0x10000000 as *const u32;
    rom_data::memcpy(boot2.as_mut_ptr() as _, xip_base as _, 256);

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    connect_internal_flash();
    flash_exit_xip();

    set_cs(false);

    let ssi = &*pac::XIP_SSI::ptr();

    for b in txrxbuf {
        while !ssi.sr.read().tfnf().bit_is_set() {}
        ssi.dr0.write(|w| w.dr().bits(*b as _));

        while !ssi.sr.read().rfne().bit_is_set() {}
        *b = ssi.dr0.read().dr().bits() as _;
    }

    set_cs(true);

    flash_flush_cache();

    let ptr = (boot2.as_mut_ptr() as *const u8).add(1) as *const ();
    let start: extern "C" fn() = core::mem::transmute(ptr);
    start();
}

fn read_uid() -> u64 {
    const FLASH_RUID_CMD: u8 = 0x4b;
    const FLASH_RUID_DUMMY_BYTES: usize = 4;
    const FLASH_RUID_DATA_BYTES: usize = 8;
    const FLASH_RUID_TOTAL_BYTES: usize = 1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES;

    let mut buf = [0; FLASH_RUID_TOTAL_BYTES];
    buf[0] = FLASH_RUID_CMD;

    unsafe {
        // do_flash_cmd(&mut buf);
        // do_flash_cmd2(buf.as_mut_ptr(), buf.len() as _);
        RAMFUNC.run(&mut buf);
    }
    // cortex_m::interrupt::free(|_| RAMFUNC.run(&mut buf));

    u64::from_le_bytes(buf[FLASH_RUID_DUMMY_BYTES + 1..].try_into().unwrap())
}

// #[repr(align(2))]
// struct FlashRunner([u8; include_bytes!("../test.bin").len()]);
//
// impl FlashRunner {
//     pub fn run(&self, txrxbuf: &mut [u8]) {
//         let run: extern "C" fn(txrxbuf: *mut u8, len: u32) =
//             unsafe { core::mem::transmute(self.0.as_ptr().add(1)) };
//
//         run(txrxbuf.as_mut_ptr(), txrxbuf.len() as _);
//     }
// }

#[link_section = ".data"]
static RAMFUNC: FlashRunner = FlashRunner(*include_bytes!("../test.bin"));

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
