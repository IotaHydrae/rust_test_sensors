#![allow(dead_code)] // register map

extern crate byteorder;
extern crate docopt;
extern crate i2cdev;

// use docopt::Docopt;
// use std::env::{self, args};
// use std::num::ParseIntError;
use sensors::apds9930::{Apds9930Proximity, APDS9930_I2C_ADDR};
use sensors::ap3216::{Ap3216Proximity, AP3216_I2C_ADDR};
use sensors::sk33562::{Sk33562Proximity, SK33562_I2C_ADDR};
use sensors::ltr553als::{Ltr553alsProximity, LTR553ALS_I2C_ADDR};
use sensors::Proximity;
use std::thread::{self, JoinHandle};
use std::time::Duration;

#[cfg(any(target_os = "linux", target_os = "android"))]
use i2cdev::core::*;
use i2cdev::linux::*;

mod sensors {
    use std::error::Error;

    pub trait Proximity {
        type Error: Error;

        fn get_proximity(&mut self) -> Result<u16, Self::Error>;
    }

    pub mod ltr553als {
        use std::thread;
        use std::time::Duration;
        use i2cdev::core::I2CDevice;

        use super::Proximity;

        pub const LTR553ALS_I2C_ADDR: u16 = 0x23;

        pub struct Ltr553alsProximity<T> {
            i2cdev: T,
        }

        impl<T> Ltr553alsProximity<T>
        where
            T: I2CDevice + Sized,
        {
            fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), T::Error> {
                self.i2cdev.smbus_write_byte_data(reg, data)
            }
            fn read_reg(&mut self, reg: u8)-> Result<u8, T::Error> {
                self.i2cdev.smbus_read_byte_data(reg)
            }

            pub fn new(i2cdev: T) -> Result<Ltr553alsProximity<T>, T::Error> {
                let mut ltr553als = Ltr553alsProximity { i2cdev };
                ltr553als.write_reg(0x81, 0x03)?;
                ltr553als.write_reg(0x83, 0x0f)?;
                Ok(ltr553als)
            }
        }

        impl<T> Proximity for Ltr553alsProximity<T>
        where
            T: I2CDevice + Sized,
        {
            type Error = T::Error;

            fn get_proximity(&mut self) -> Result<u16, Self::Error> {
                let ps_data_0 = self.read_reg(0x8d)?;
                let ps_data_1 = self.read_reg(0x8e)?;
                Ok((((ps_data_1 & 0x7) as u16) << 8) | ps_data_0 as u16)
            }
        }
    }

    pub mod sk33562 {
        use std::thread;
        use std::time::Duration;
        use i2cdev::core::I2CDevice;

        use super::Proximity;

        pub const SK33562_I2C_ADDR: u16 = 0x46;

        pub struct Sk33562Proximity<T> {
            i2cdev: T,
        }

        impl<T> Sk33562Proximity<T>
        where
            T: I2CDevice + Sized,
        {
            fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), T::Error> {
                self.i2cdev.smbus_write_byte_data(reg, data)
            }
            fn read_reg(&mut self, reg: u8) -> Result<u8, T::Error> {
                self.i2cdev.smbus_read_byte_data(reg)
            }

            pub fn new(i2cdev: T) -> Result<Sk33562Proximity<T>, T::Error> {
                let mut sk33562 = Sk33562Proximity { i2cdev };
                sk33562.write_reg(0x80, 0x01)?;
                thread::sleep(Duration::from_millis(50));
                sk33562.write_reg(0x0, 0x03)?;
                sk33562.write_reg(0x01, 0x3 << 4)?;
                sk33562.write_reg(0x3, 0xA << 4)?;
                sk33562.write_reg(0x6, 0)?;
                sk33562.write_reg(0x7, 0)?;
                Ok(sk33562)
            }

            pub fn get_als_data(&mut self) -> Result<u16, T::Error> {
                let reg_als_low = self.read_reg(0x13).unwrap();
                let reg_als_high = self.read_reg(0x14).unwrap();

                Ok((reg_als_low as  u16) << 8 | reg_als_high as u16)
            }
        }

        impl<T> Proximity for Sk33562Proximity<T>
        where
            T: I2CDevice + Sized,
        {
            type Error = T::Error;

            fn get_proximity(&mut self) -> Result<u16, Self::Error> {
                let msb = self.read_reg(0x11)?;
                let lsb = self.read_reg(0x12)?;
                Ok((msb as  u16) << 8 | lsb as u16)
                // Ok(lsb as u16)
            }
        }
    }

    pub mod ap3216 {
        use std::thread;
        use std::time::Duration;
        use i2cdev::core::I2CDevice;

        use super::Proximity;

        pub const AP3216_I2C_ADDR: u16 = 0x1e;

        pub struct Ap3216Proximity<T> {
            flag_object: bool,
            i2cdev: T,
        }

        impl<T> Ap3216Proximity<T>
        where
            T: I2CDevice + Sized,
        {
            fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), T::Error> {
                self.i2cdev.smbus_write_byte_data(reg, data)
            }
            fn read_reg(&mut self, reg: u8) -> Result<u8, T::Error> {
                self.i2cdev.smbus_read_byte_data(reg)
            }

            pub fn new(i2cdev: T) -> Result<Ap3216Proximity<T>, T::Error> {
                let mut ap3216_dev = Ap3216Proximity { i2cdev, flag_object: false };
                /* set system mode to power down */
                ap3216_dev.write_reg(0, 0)?;

                /* set system mode, ALS and PS+IR function active */
                ap3216_dev.write_reg(0, 0x3)?;
                ap3216_dev.write_reg(0x20, 0x3 << 2)?;
                ap3216_dev.write_reg(0x21, 0x3 << 4)?;
                thread::sleep(Duration::from_millis(14));
                Ok(ap3216_dev)
            }

            pub fn get_ir_data(&mut self) -> Result<u16, T::Error> {
                let reg_ir_low = self.read_reg(0x0a).unwrap();
                let reg_ir_high = self.read_reg(0x0b).unwrap();

                Ok((((reg_ir_high as u16) & 0xFF) << 2) | ((reg_ir_low as u16) & 0x3))
            }

            pub fn get_als_data(&mut self) -> Result<u16, T::Error> {
                let reg_als_low = self.read_reg(0x0c).unwrap();
                let reg_als_high = self.read_reg(0x0d).unwrap();

                Ok((((reg_als_high as u16) & 0xFF) << 8) | ((reg_als_low as u16) & 0xFF))
            }

            pub fn is_obj_detected(&mut self) -> bool {
                self.flag_object
            }
        }

        impl<T> Proximity for Ap3216Proximity<T>
        where
            T: I2CDevice + Sized,
        {
            type Error = T::Error;

            #[doc = "proximity is a 10 bit register"]
            fn get_proximity(&mut self) -> Result<u16, Self::Error> {
                let reg_ps_low = self.read_reg(0x0e).unwrap();
                let reg_ps_high = self.read_reg(0x0f).unwrap();

                self.flag_object = (reg_ps_high & 0x80) != 0;

                Ok((((reg_ps_high as u16) & 0x3F) << 4) | ((reg_ps_low as u16) & 0xf))
            }
        }
    }

    pub mod apds9930 {
        use std::thread;
        use std::time::Duration;
        use i2cdev::core::I2CDevice;

        use super::Proximity;

        pub const APDS9930_I2C_ADDR: u16 = 0x39;
        const ATIME: u8 = 0xff;
        const WTIME: u8 = 0xff;
        const PTIME: u8 = 0xff;
        const PPULSE: u8 = 1;
        const PDRIVE: u8 = 0;
        const PDIODE: u8 = 0x20;
        const PGAIN: u8 = 0;
        const AGAIN: u8 = 0;
        const WEN: u8 = 8;
        const PEN: u8 = 4;
        const AEN: u8 = 2;
        const PON: u8 = 1;

        pub struct Apds9930Proximity<T> {
            i2cdev: T,
        }

        impl<T> Apds9930Proximity<T>
        where
            T: I2CDevice + Sized,
        {
            fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), T::Error> {
                self.i2cdev.smbus_write_byte_data(0x80 | reg, data)
            }
            fn read_word_data(&mut self, reg: u8) -> Result<u16, T::Error> {
                Ok(self.i2cdev.smbus_read_word_data(0xA0 | reg)?)
            }

            fn read_byte_data(&mut self, reg: u8) -> Result<u8, T::Error> {
                Ok(self.i2cdev.smbus_read_byte_data(0xA0 | reg)?)
            }
            pub fn new(i2cdev: T) -> Result<Apds9930Proximity<T>, T::Error> {
                let mut apds9930_dev = Apds9930Proximity { i2cdev };
                apds9930_dev.write_reg(0, 0)?;
                apds9930_dev.write_reg(1, ATIME)?;
                apds9930_dev.write_reg(2, PTIME)?;
                apds9930_dev.write_reg(3, WTIME)?;
                apds9930_dev.write_reg(0xe, PPULSE)?;

                apds9930_dev.write_reg(0xf, PDRIVE | PDIODE | PGAIN | AGAIN)?;
                apds9930_dev.write_reg(0, WEN | PEN | AEN | PON)?;
                thread::sleep(Duration::from_millis(12));
                Ok(apds9930_dev)
            }

            pub fn get_ch0_data(&mut self) -> Result<u16, T::Error> {
                self.read_word_data(0x14)
            }

            pub fn get_ch1_data(&mut self) -> Result<u16, T::Error> {
                self.read_word_data(0x16)
            }

            #[doc = "Set the gain of proximity, available values: 1, 2, 4, 8"]
            pub fn set_pgain(&mut self, gain: u8) -> Result<(), T::Error> {
                let available_gains = [1, 2, 4, 8];
                if !available_gains.contains(&gain) {
                    panic!("Invalid pgain value! {:?}, available values: {:?}", gain, available_gains)
                }
                println!("Setting pgain to {}, {}", gain, (((gain as f64).log2() as u8)));
                let reg_val = self.read_byte_data(0xf)?;
                self.write_reg(0xf, reg_val | (((gain as f64).log2() as u8) << 2))?;
                Ok(())
            }

            pub fn get_pgain(&mut self) -> Result<u8, T::Error> {
                let reg_val = self.read_byte_data(0xf)?;
                let pgain = (2 as i32).pow(((reg_val >> 2) & 0x3)as u32);
                Ok(pgain as u8)
            }

            #[doc = "Set the gain of ALS, available values: 1, 8, 16, 120"]
            pub fn set_again(&mut self, gain: u8) -> Result<(), T::Error> {
                let available_gains = [1, 8, 16, 120];
                if !available_gains.contains(&gain) {
                    panic!("Invalid again value! {:?}, available values: {:?}", gain, available_gains)
                }
                let again = match gain {
                    1 => 0,
                    8 => 1,
                    16 => 2,
                    120 => 3,
                    _ => unreachable!(),
                };
                println!("Setting again to {}, {}", gain, again);
                let reg_val = self.read_byte_data(0xf)?;
                self.write_reg(0xf, reg_val | (again &  0x3) << 0)?;
                Ok(())
            }

            pub fn get_again(&mut self) -> Result<u8, T::Error> {
                let reg_val = self.read_byte_data(0xf)?;
                let again = match reg_val & 0x3 {
                    0 => 1,
                    1 => 8,
                    2 => 16,
                    3 => 120,
                    _ => unreachable!(),
                };
                Ok(again as u8)
            }
        }

        impl<T> Proximity for Apds9930Proximity<T>
        where
            T: I2CDevice + Sized,
        {
            type Error = T::Error;

            fn get_proximity(&mut self) -> Result<u16, Self::Error> {
                self.read_word_data(0x18)
            }
        }
    }
}

// ifconfig eth0 up && udhcpc -i eth0
// mount -t nfs -o nolock,vers=3 192.168.50.184:/home/iotah/.nfs_share /mnt
// echo "1 4 1 7" > /proc/sys/kernel/printk

fn main() {
    let apds9930_i2cdev = LinuxI2CDevice::new("/dev/i2c-3", APDS9930_I2C_ADDR).unwrap();
    let apds9930_dev = Apds9930Proximity::new(apds9930_i2cdev);
    match apds9930_dev {
        Ok(mut dev) => {
            println!("APDS-9930@0x{:x} found!", APDS9930_I2C_ADDR);
            dev.set_pgain(8).unwrap();
            dev.set_again(120).unwrap();

            thread::spawn(move || {
                loop {
                    println!("[APDS-9930@0x{:x}] ch0: {}", APDS9930_I2C_ADDR, dev.get_ch0_data().unwrap());
                    println!("[APDS-9930@0x{:x}] ch1: {}", APDS9930_I2C_ADDR, dev.get_ch1_data().unwrap());
                    println!("[APDS-9930@0x{:x}] proximity: {}", APDS9930_I2C_ADDR, dev.get_proximity().unwrap());
                    thread::sleep(Duration::from_millis(100));
                }
            });
        }
        Err(e) => println!("[APDS-9930@0x{:x}] No such device!, {:?}", APDS9930_I2C_ADDR, e),
    };


    let ap3216_i2cdev = LinuxI2CDevice::new("/dev/i2c-3", AP3216_I2C_ADDR).unwrap();
    let ap3216_dev = Ap3216Proximity::new(ap3216_i2cdev);
    match ap3216_dev {
        Ok(mut dev) => {
            println!("[AP3216@0x{:x}] found!", AP3216_I2C_ADDR);

            thread::spawn(move || {
                loop {
                    println!("[AP3216@0x{:x}] ir: {}", AP3216_I2C_ADDR, dev.get_ir_data().unwrap());
                    println!("[AP3216@0x{:x}] als: {}", AP3216_I2C_ADDR, dev.get_als_data().unwrap());
                    println!("[AP3216@0x{:x}] proximity: {}", AP3216_I2C_ADDR, dev.get_proximity().unwrap());

                    println!("object detected: {}", dev.is_obj_detected());

                    thread::sleep(Duration::from_millis(100));
                }
            });
        }
        Err(e) => println!("[AP3216@0x{:x}] No such device!, {:?}", AP3216_I2C_ADDR, e),
    }

    let sk33562_i2cdev = LinuxI2CDevice::new("/dev/i2c-3", SK33562_I2C_ADDR).unwrap();
    let sk33562_dev = Sk33562Proximity::new(sk33562_i2cdev);
    match sk33562_dev {
        Ok(mut dev) => {
            println!("[SK33562@0x{:x}] found!", SK33562_I2C_ADDR);

            thread::spawn(move || {
                loop {
                    println!("[SK33562@0x{:x}] als: {}", SK33562_I2C_ADDR, dev.get_als_data().unwrap());
                    println!("[SK33562@0x{:x}] proximity: {}", SK33562_I2C_ADDR, dev.get_proximity().unwrap());

                    // println!("object detected: {}", dev.is_obj_detected());

                    thread::sleep(Duration::from_millis(100));
                }
            });
        }
        Err(e) => println!("[SK33562@0x{:x}] No such device!, {:?}", SK33562_I2C_ADDR, e),
    }

    let ltr553als_i2cdev = LinuxI2CDevice::new("/dev/i2c-3", LTR553ALS_I2C_ADDR).unwrap();
    let ltr553als_dev = Ltr553alsProximity::new(ltr553als_i2cdev);
    match ltr553als_dev {
        Ok(mut dev) => {
            println!("[LTR553ALS@0x{:x}] found!", LTR553ALS_I2C_ADDR);

            thread::spawn(move || {
                loop {
                    // println!("[LTR553ALS@0x{:x}] als: {}", SK33562_I2C_ADDR, dev.get_als_data().unwrap());
                    println!("[LTR553ALS@0x{:x}] proximity: {}", LTR553ALS_I2C_ADDR, dev.get_proximity().unwrap());

                    // println!("object detected: {}", dev.is_obj_detected());

                    thread::sleep(Duration::from_millis(100));
                }
            });
        }
        Err(e) => println!("[LTR553ALS@0x{:x}] No such device!, {:?}", LTR553ALS_I2C_ADDR, e),
    }

    loop {
        thread::sleep(Duration::from_millis(100));
    }
}
