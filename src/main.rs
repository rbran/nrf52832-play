#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::asm;
use rtt_target::{rprintln, rtt_init_print};

use nrf52832_pac as pac;

fn delay_ms(time: u32) {
    for _ in 0..time {
        delay_us(1000);
    }
}

fn delay_us(time: u32) {
    asm::delay(time * 16);
}

const ADV_ADDRESS: u32 = 0x8E89BED6;

#[cortex_m_rt::entry]
fn main() -> ! {
    unsafe {
        let clock = pac::CLOCK::ptr();
        //the 16Mhz external crustal
        (*clock).events_hfclkstarted.write(|w| w.bits(0));
        (*clock).tasks_hfclkstart.write(|w| w.bits(1));
        while (*clock).events_hfclkstarted.read().bits() == 0 {}
    }
    rtt_init_print!();

    let mut buf = [0u8; 37 + 4];
    buf[1] = (buf.len() - 2) as u8; //update length
    const TX_BUF: &[u8] = &[
        0,  //S0
        28, //Length
        //data from real device
        0x9f, 0x84, 0x4a, 0x8b, 0x98, 0xcc, 0x02, 0x01, 0x02, 0x03, 0x03, 0x03,
        0xfe, 0x0e, 0x09, 0x4c, 0x45, 0x5f, 0x57, 0x48, 0x2d, 0x31, 0x31, 0x31,
        0x31, 0x58, 0x4d, 0x33,
    ];

    let tx_pkt = |buf: &mut [u8]| {
        TX_BUF.iter().enumerate().for_each(|(i, &x)| buf[i] = x);
    };

    rprintln!("Radio is being configured");
    let radio = pac::RADIO::ptr();
    let radio_state = || unsafe { (*radio).state.read().state() };
    let radio_state_print = || {
        rprintln!("Radio state: {:?}", radio_state().variant());
    };
    let radio_events_clr = || unsafe {
        (*radio).events_ready.write(|w| w.bits(0));
        (*radio).events_address.write(|w| w.bits(0));
        (*radio).events_payload.write(|w| w.bits(0));
        (*radio).events_end.write(|w| w.bits(0));
        (*radio).events_disabled.write(|w| w.bits(0));
        (*radio).events_devmatch.write(|w| w.bits(0));
        (*radio).events_devmiss.write(|w| w.bits(0));
        (*radio).events_rssiend.write(|w| w.bits(0));
        (*radio).events_bcmatch.write(|w| w.bits(0));
        (*radio).events_crcok.write(|w| w.bits(0));
        (*radio).events_crcerror.write(|w| w.bits(0));
    };
    let radio_events_print = || unsafe {
        rprintln!("Radio ready: {:?}", (*radio).events_ready.read().bits());
        rprintln!("Radio addr: {:?}", (*radio).events_address.read().bits());
        rprintln!("Radio payload: {:?}", (*radio).events_payload.read().bits());
        rprintln!("Radio end: {:?}", (*radio).events_end.read().bits());
        rprintln!("Radio disabled: {:?}", (*radio).events_disabled.read().bits());
        rprintln!("Radio devmatch: {:?}", (*radio).events_devmatch.read().bits());
        rprintln!("Radio devmiss: {:?}", (*radio).events_devmiss.read().bits());
        rprintln!("Radio rssiend: {:?}", (*radio).events_rssiend.read().bits());
        rprintln!("Radio bcmatch: {:?}", (*radio).events_bcmatch.read().bits());
        rprintln!("Radio crcok: {:?}", (*radio).events_crcok.read().bits());
        rprintln!("Radio crerror: {:?}", (*radio).events_crcerror.read().bits());
    };

    let radio_txen = || unsafe { (*radio).tasks_txen.write(|w| w.bits(1)) };
    let radio_rxen = || unsafe { (*radio).tasks_rxen.write(|w| w.bits(1)) };

    let radio_start = || unsafe { (*radio).tasks_start.write(|w| w.bits(1)) };
    let radio_disable =
        || unsafe { (*radio).tasks_disable.write(|w| w.bits(1)) };

    let radio_stop = || unsafe { (*radio).tasks_stop.write(|w| w.bits(1)) };

    let radio_ready_clr =
        || unsafe { (*radio).events_ready.write(|w| w.bits(0)) };
    let radio_ready = || unsafe { (*radio).events_ready.read().bits() != 0 };

    let radio_end_clr = || unsafe { (*radio).events_end.write(|w| w.bits(0)) };
    let radio_end = || unsafe { (*radio).events_end.read().bits() != 0 };

    let radio_disabled_clr =
        || unsafe { (*radio).events_disabled.write(|w| w.bits(0)) };
    let radio_disabled =
        || unsafe { (*radio).events_disabled.read().bits() != 0 };

    let radio_on = || unsafe {
        (*radio).shorts.write(|w| w.bits(0));
        radio_disabled_clr();
        radio_disable();
        while !radio_disabled() {}
        (*radio).power.write(|w| w.bits(1));
    };
    let radio_off = || unsafe {
        (*radio).power.write(|w| w.bits(0));
    };
    let radio_ptr = |buf: &[u8]| unsafe {
        //set addr prt
        let output = core::mem::transmute::<*const u8, u32>(buf.as_ptr());
        //if output % 4 != 0 {
        //    panic!("Unable to align");
        //}
        (*radio).packetptr.write(|w| w.bits(output));
    };
    let radio_cnf = || unsafe {
        //mode is BLE 1Mbit
        (*radio).mode.write(|w| w.mode().ble_1mbit());
        //set address, BASE0 + PREFIX0.AP0
        (*radio).prefix0.write(|w| w.bits(ADV_ADDRESS >> 24));
        (*radio).base0.write(|w| w.bits(ADV_ADDRESS << 8));
        //use logical addr 0: BASE0 + PREFIX0.AP0
        (*radio).rxaddresses.write(|w| w.addr0().set_bit());
        //transmit logical addr 0: BASE0 + PREFIX0.AP0
        (*radio).txaddress.write(|w| w.bits(0));
        (*radio).base0.write(|w| w.bits(ADV_ADDRESS << 8));
        //set channel 37, 2402Mhz
        (*radio).frequency.write(|w| w.bits(2));
        //tx power
        (*radio).txpower.write(|w| w.txpower()._0d_bm());
        //disable shorts for now
        (*radio).shorts.write(|w| w.bits(0));
        //configs
        (*radio).pcnf0.write(|w| {
            w.lflen()
                .bits(8) //len is 6 bits
                .s0len()
                .set_bit() //s0 is enabled (1 byte)
                .s1len()
                .bits(0) //s1 0 bites
                .s1incl()
                .clear_bit() //auto include s1 in ram
                .plen()
                ._8bit() //preample is 1 byte
        });
        (*radio).pcnf1.write(|w| {
            w.maxlen()
                .bits(37) //max len is 37
                .statlen()
                .bits(0) //static lenght 0
                .balen()
                .bits(3) //base address is 3 bytes
                .endian()
                .little() //little endian
                .whiteen()
                .enabled() //whiteen enabled
        });
        //set the CRC to BLE
        (*radio).crccnf.write(|w| w.len().three().skipaddr().skip());
        (*radio).crcpoly.write(|w| w.bits(0x65b));
        (*radio).crcinit.write(|w| w.bits(0x00555555));
        //spacing 150ms
        (*radio).tifs.write(|w| w.bits(150));
        //whiteiv on channel 37
        (*radio).datawhiteiv.write(|w| w.bits(37));
    };
    //put radio in rx mode
    //let radio_rxen = || unsafe { (*radio).tasks_rxen.write(|w| w.bits(1)) };
    radio_on();
    rprintln!("Radio is on");
    radio_cnf();
    //rprintln!("Radio is cnf");
    radio_ptr(&buf);

    if false {
        //tx some pkts
        tx_pkt(&mut buf);
        for _ in 0..30 {
            //turn on the device
            radio_ready_clr();
            radio_txen();
            while !radio_ready() {}
            rprintln!("Radio TX is ready");

            //start transmission
            radio_end_clr();
            radio_start();
            while !radio_end() {}
            rprintln!("Radio TX is done");

            //disable radio
            radio_disabled_clr();
            radio_disable();
            while !radio_disabled() {}
            rprintln!("Radio TX disabled");

            delay_ms(10);
        }
    }


    //rx some pkts
    if true {
        for _ in 0..30 {
            //turn on the device
            radio_ready_clr();
            radio_rxen();
            while !radio_ready() {}
            rprintln!("Radio RX is ready");

            //start transmission
            //radio_end_clr();
            radio_end_clr();
            radio_start();
            while !radio_end() {
            }
            rprintln!("RX PKT {:02x?}", buf);

            //disable radio
            radio_disabled_clr();
            radio_disable();
            while !radio_disabled() {}
            rprintln!("Radio RX disabled");
            delay_ms(10);
        }
    }

    rprintln!("Radio is {:?}", radio_state().variant());

    //const LED_PIN: usize = 11;
    //let p0 = pac::P0::ptr();
    //unsafe {
    //    (*p0).pin_cnf[LED_PIN].write(|w| w.dir().output()
    //                                 .input().disconnect()
    //                                 .pull().disabled()
    //                                 .drive().s0s1()
    //                                 .sense().disabled());
    //}
    //let led_low = || unsafe { (*p0).outclr.write(|w| w.bits(1u32 << LED_PIN)) };
    //let led_high = || unsafe { (*p0).outset.write(|w| w.bits(1u32 << LED_PIN)) };
    //for _ in 0..10 {
    //    delay_ms(1000);
    //    rprintln!("led on");
    //    led_high();
    //    delay_ms(1000);
    //    rprintln!("led off");
    //    led_low();
    //}

    panic!("This is an intentional panic to break the loop.");
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {
        cortex_m::asm::bkpt();
    }
}
