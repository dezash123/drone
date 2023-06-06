#![no_std]
#![no_main]
use panic_halt as _;
use rp_pico::entry;
#[entry]
fn see_doesnt_have_to_be_called_main() -> ! {
    loop {}
}

