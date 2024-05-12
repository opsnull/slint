// 参考：
// gt911: https://github.com/enelson1001/rust-esp32s3-lvgl-clickme/blob/master/src/gt911.rs
// tt21100 的驱动实现：https://github.com/jessebraham/tt21100/blob/main/tt21100/src/lib.rs
// tt21100 的例子：https://github.com/sambenko/esp-box-tt21100-example/blob/main/src/main.rs
// GT911 寄存器列表：https://github.com/STMicroelectronics/stm32-gt911/blob/main/gt911_reg.h

// 说明：
// 1. ESP32-S3-BOX-3 的 GT911 I2C 地址是 0x14，而非默认的 0x5d（测试发现）；参考：
//   https://github.com/espressif/esp-bsp/blob/master/components/lcd_touch/esp_lcd_touch_gt911/include/esp_lcd_touch_gt911.h#L34C1-L40C4
// 2. 使用较老版本的 embedded_hal 库（ 0.2.5 版本，而非新的 1.0.0 版本），从而与其他库兼容；
// 3. 由于 Touch 和 LCD 共享 reset 引脚，而 LCD 初始化时已经设置了 reset，故 Touch 不需要 reset，去掉了相关逻辑，否则会导致 LCD 显示白屏；
// 4. 自定义 Error，封装其他类型 Error，便于报错；
// 5. 新增 IRQ 引脚和基于 irq 引脚的 data_available() 方法，后续 slint 调用该方法来判断是否有触摸事件；


// 已知问题：
// 1. 一次触摸后产生多次触摸 event（参考中断日志）导致一些按钮/选择逻辑不对；

use core::{array::TryFromSliceError, fmt::Debug};

/// A minimal implementation of the GT911 to work with Lvgl since Lvgl only uses a single touch point
/// The default orientation and size are based on the aliexpress ESP 7 inch capactive touch development
/// board model ESP-8048S070C
use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::InputPin,
};

// 可能是两个地址：0x5d、0x14，box 开发版是 0x14，如果使用出错则

// !! A panic occured in 'examples/mcu-board-support/esp32_s3_box.rs', at line 205, column 33

// PanicInfo {
//     payload: Any { .. },
//     message: Some(
//         called `Result::unwrap()` on an `Err` value: BusError(AckCheckFailed),
//     ),
//     location: Location {
//         file: "examples/mcu-board-support/esp32_s3_box.rs",
//         line: 205,
//         col: 33,
//     },
//     can_unwind: true,
//     force_no_backtrace: false,
// }

// Backtrace:

const DEFAULT_GT911_ADDRESS: u8 = 0x14; // S3-Box-3 使用的是 0x14，而非 0x5d；

/// Any type of error which may occur while interacting with the device
#[derive(Debug)]
pub enum Error<E> {
    /// Some error originating from the communication bus
    BusError(E),
    /// The message length did not match the expected value
    InvalidMessageLen(usize),
    /// Reading a GPIO pin resulted in an error
    IOError,
    /// Tried to read a touch point, but no data was available
    NoDataAvailable,
    /// Error converting a slice to an array
    TryFromSliceError,
}

impl<E> From<TryFromSliceError> for Error<E> {
    fn from(_: TryFromSliceError) -> Self {
        Self::TryFromSliceError
    }
}

/// Documented registers of the device
#[allow(dead_code)]
#[repr(u16)]
#[derive(Debug, Clone, Copy)]
enum Reg {
    ProductId = 0x8140,
    PointInfo = 0x814E,
    Point1 = 0x814F,
}

/// Represents the orientation of the device
#[derive(Copy, Clone, Debug)]
pub enum Orientation {
    Portrait, // Do Not use
    Landscape,
    InvertedPortrait, // Do Not use
    InvertedLandscape,
}

/// Represents the dimensions of the device
#[derive(Copy, Clone, Debug)]
pub struct Dimension {
    pub height: u16,
    pub width: u16,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TouchPoint {
    pub id: u8,
    pub x: u16,
    pub y: u16,
    pub size: u16,
}

/// Driver representation holding:
///
/// - The I2C Slave address of the GT911
/// - The I2C Bus used to communicate with the GT911
/// - The reset pin on the GT911
/// - The delay used by the reset pin to reset the GT911
/// - The screen/panel orientation
/// - The scree/panel dimesions
#[derive(Clone, Debug)]
pub struct GT911<I2C, IRQ> {
    address: u8,
    i2c: I2C,
    irq_pin: IRQ,
    orientation: Orientation,
    size: Dimension,
}

impl<I2C, IRQ, E> GT911<I2C, IRQ>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    IRQ: InputPin,
    E: Debug,
{
    pub fn new(i2c: I2C, irq_pin: IRQ) -> Self {
        Self {
            address: DEFAULT_GT911_ADDRESS,
            i2c,
            irq_pin,
            orientation: Orientation::Landscape,
            size: Dimension { height: 240, width: 320 }, // 320*240
        }
    }

    pub fn data_available(&self) -> Result<bool, Error<E>> {
        self.irq_pin.is_low().map_err(|_| Error::IOError)
    }

    // pub fn reset(&mut self) -> Result<(), Error<E>> {
    //     //println!("======= Resetting GT911 =======");
    //     self.reset_pin.set_low().map_err(|_| Error::IOError)?;
    //     self.delay.delay_us(100);
    //     self.reset_pin.set_high().map_err(|_| Error::IOError)?;
    //     self.delay.delay_ms(100);

    //     Ok(())
    // }

    pub fn set_orientation(&mut self, orientation: Orientation) {
        self.orientation = orientation;
    }

    pub fn set_size(&mut self, height: u16, width: u16) {
        self.size = Dimension { height, width };
    }

    // no_std 不支持 String 和 std 库。
    // // Useful function to determine if you are communicating with GT911, The GT911 must first be reset.
    // // The return string should be - 911
    pub fn read_product_id(&mut self) -> Result<(), Error<E>> {
        let mut rx_buf: [u8; 4] = [0; 4];

        let product_id_reg: u16 = Reg::ProductId as u16;

        let hi_byte: u8 = (product_id_reg >> 8).try_into().unwrap();
        let lo_byte: u8 = (product_id_reg & 0xFF).try_into().unwrap();
        let tx_buf: [u8; 2] = [hi_byte, lo_byte];

        esp_println::println!("read_product_id 1: {:?}", &rx_buf);
        self.i2c.write_read(self.address, &tx_buf, &mut rx_buf).map_err(|e| Error::BusError(e))?;
        esp_println::println!("read_product_id 2: {:?}", &rx_buf);
        Ok(())
    }

    pub fn read_touch(&mut self) -> Result<Option<TouchPoint>, Error<E>> {
        let mut rx_buf: [u8; 1] = [0xFF];

        let point_info_reg: u16 = Reg::PointInfo as u16;
        let hi_byte: u8 = (point_info_reg >> 8).try_into().unwrap();
        let lo_byte: u8 = (point_info_reg & 0xFF).try_into().unwrap();
        let tx_buf: [u8; 2] = [hi_byte, lo_byte];

        //esp_println::println!("read_touch 1: {:?} {:?} {:?}", self.address, tx_buf, rx_buf);

        self.i2c.write_read(self.address, &tx_buf, &mut rx_buf).map_err(|e| Error::BusError(e))?;

        let point_info = rx_buf[0];
        let buffer_status = point_info >> 7 & 1u8;
        let touches = point_info & 0x7;

        // esp_println::println!("point info = {:x?}", point_info);
        // esp_println::println!("bufferStatus = {:?}", point_info >> 7 & 1u8);
        // esp_println::println!("largeDetect = {:?}", point_info >> 6 & 1u8);
        // esp_println::println!("proximityValid = {:?}", point_info >> 5 & 1u8);
        // esp_println::println!("HaveKey = {:?}", point_info >> 4 & 1u8);
        // esp_println::println!("touches = {:?}", point_info & 0xF);

        let is_touched: bool = buffer_status == 1 && touches > 0;

        let mut tp: TouchPoint = TouchPoint { id: 0, x: 0, y: 0, size: 0 };

        if is_touched {
            tp = self.read_touch_point(Reg::Point1 as u16).map_err(|_| Error::IOError)?;
        }

        // Reset point_info register after reading it
        let tx_buf: [u8; 3] = [hi_byte, lo_byte, 0u8];
        self.i2c.write(self.address, &tx_buf).map_err(|e| Error::BusError(e))?;

        Ok(if is_touched { Some(tp) } else { None })
    }

    pub fn read_touch_point(&mut self, point_register: u16) -> Result<TouchPoint, Error<E>> {
        let hi_byte: u8 = (point_register >> 8).try_into().unwrap();
        let lo_byte: u8 = (point_register & 0xFF).try_into().unwrap();
        let tx_buf: [u8; 2] = [hi_byte, lo_byte];

        let mut rx_buf: [u8; 7] = [0; 7];
        self.i2c.write_read(self.address, &tx_buf, &mut rx_buf).map_err(|e| Error::BusError(e))?;

        let id: u8 = rx_buf[0];
        let mut x: u16 = rx_buf[1] as u16 + ((rx_buf[2] as u16) << 8);
        let mut y: u16 = rx_buf[3] as u16 + ((rx_buf[4] as u16) << 8);
        let size: u16 = rx_buf[5] as u16 + ((rx_buf[6] as u16) << 8);

        //println!("========== x = {:?}    y = {:?} ==========", x, y);

        match self.orientation {
            Orientation::Landscape => {
                // Don't need to do anything because x = x and y = y
            }
            Orientation::Portrait => {
                let temp: u16 = x;
                x = y;
                y = self.size.height - temp;
            }
            Orientation::InvertedLandscape => {
                x = self.size.width - x;
                y = self.size.height - y;
            }
            Orientation::InvertedPortrait => {
                let temp: u16 = x;
                x = self.size.width - y;
                y = temp;
            }
        }

        Ok(TouchPoint { id, x, y, size })
    }
}
