// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_graphics_core::{pixelcolor::Rgb565, prelude::*};

use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    prelude::{DrawTarget, Point, RgbColor},
    text::{Alignment, Text},
    Drawable,
};

use embedded_hal::digital::v2::OutputPin;
use esp32s3_hal::{
    clock::{ClockControl, CpuClock},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
    systimer::SystemTimer,
    timer::TimerGroup,
    Delay, Rtc, IO,
};
use esp_alloc::EspHeap;
use esp_backtrace as _;
use mipidsi::{Display, Orientation};
use slint::platform::WindowEvent;
pub use xtensa_lx_rt::entry;

use crate::gt911::{self, GT911};

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

pub fn init() {
    const HEAP_SIZE: usize = 250 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as *mut u8, HEAP_SIZE) }
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");
}

#[derive(Default)]
struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

// https://github.com/almindor/mipidsi/issues/73
struct NoPin;
impl embedded_hal::digital::v2::OutputPin for NoPin {
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() / (SystemTimer::TICKS_PER_SECOND / 1000),
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let mut system = peripherals.SYSTEM.split();
        let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        let timer_group0 =
            TimerGroup::new(peripherals.TIMG0, &clocks, &mut system.peripheral_clock_control);
        let mut wdt0 = timer_group0.wdt;
        let timer_group1 =
            TimerGroup::new(peripherals.TIMG1, &clocks, &mut system.peripheral_clock_control);
        let mut wdt1 = timer_group1.wdt;

        rtc.rwdt.disable();
        wdt0.disable();
        wdt1.disable();

        let mut delay = Delay::new(&clocks);
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

        // 引脚定义参考：https://github.com/espressif/esp-bsp/blob/master/bsp/esp-box-3/include/bsp/esp-box-3.h
        // #define BSP_I2C_SCL           (GPIO_NUM_18)
        // #define BSP_I2C_SDA           (GPIO_NUM_8)
        // #define BSP_LCD_TOUCH_INT     (GPIO_NUM_3)

        // 开发板使用的是 I2C 1
        let i2c = I2C::new(
            peripherals.I2C1,
            io.pins.gpio8,
            io.pins.gpio18,
            400u32.kHz(),
            &mut system.peripheral_clock_control,
            &clocks,
        );

        // let mut touch = tt21100::TT21100::new(i2c, io.pins.gpio3.into_pull_up_input())
        //     .expect("Initialize the touch device");

        let mut touch = gt911::GT911::new(i2c, io.pins.gpio3.into_pull_up_input());
        //touch.set_orientation(gt911::Orientation::InvertedLandscape);

        // 引脚定义参考：https://github.com/espressif/esp-bsp/blob/master/bsp/esp-box-3/include/bsp/esp-box-3.h
        let sclk = io.pins.gpio7;
        let mosi = io.pins.gpio6;
        let cs = io.pins.gpio5.into_push_pull_output();
        let dc = io.pins.gpio4.into_push_pull_output();
        let mut backlight = io.pins.gpio47.into_push_pull_output();
        let mut reset = io.pins.gpio48.into_push_pull_output();
        reset.internal_pull_up(true); // 将 reset pin 设置为正常运行所需要的 high

        // 参考：https://github.com/espressif/esp-box/issues/120#issuecomment-2063156343
        // https://github.com/sambenko/esp32s3box-display-and-publish/blob/main/src/main.rs
        // init without a RST pin： https://github.com/almindor/mipidsi/issues/73
        // https://github.com/almindor/mipidsi/pull/81
        // https://github.com/almindor/mipidsi/issues/134
        // https://github.com/almindor/mipidsi/blob/master/docs/TROUBLESHOOTING.md#reset-pin
        // 所有 LCD controller 的 reset pin 默认都是 low 有效，正常运行时必须为 high。
        // 驱动中 reset_pin() 方法可以通过软件来设置，如果不行，需要在 init() 前将 reset pin 设置为 high。
        let spi = Spi::new_no_cs_no_miso(
            peripherals.SPI2,
            sclk,
            mosi,
            40u32.MHz(),
            SpiMode::Mode0,
            &mut system.peripheral_clock_control,
            &clocks,
        );

        let di = SPIInterface::new(spi, dc, cs);
        delay.delay_ms(500u32);

        // let mut display = mipidsi::Builder::ili9342c_rgb565(di)
        //     .with_display_size(320, 240)
        //     .with_framebuffer_size(320, 240)
        //     .with_orientation(mipidsi::Orientation::PortraitInverted(false))
        //     .with_color_order(mipidsi::ColorOrder::Bgr)
        //     .init(&mut delay, None::<NoPin>) // 关键：不能使用 Some(reset) ，否则重启后显示白屏。
        //     .unwrap();
        // backlight.set_high().unwrap();

        // LCD 配置参数：https://github.com/espressif/esp-bsp/blob/master/bsp/esp-box-3/include/bsp/display.h
        let mut display = mipidsi::Builder::ili9341_rgb565(di)
            .with_display_size(320, 240)
            .with_framebuffer_size(320, 240)
            .with_orientation(mipidsi::Orientation::PortraitInverted(false))
            .with_color_order(mipidsi::ColorOrder::Bgr)
            .init(&mut delay, None::<NoPin>) // 关键：不能传入 Some(reset) 来自动设置 reset pin ，否则重启后显示白屏。
            .unwrap();

        backlight.set_high().unwrap();
        display.clear(Rgb565::WHITE).unwrap();

        let default_style =
            MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(RgbColor::BLACK).build();

        let espressif_style: MonoTextStyle<'_, Rgb565> =
            MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(RgbColor::CYAN).build();

        for position_y in (25..240).step_by(28) {
            for position_x in 0..320 {
                Text::with_alignment(
                    "O",
                    Point::new(position_x, position_y),
                    default_style,
                    Alignment::Center,
                )
                .draw(&mut display)
                .unwrap();
            }
        }

        let size = display.size();
        let size = slint::PhysicalSize::new(size.width, size.height);

        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
        };

        let mut last_touch = None;

        // touch.read_product_id().unwrap();

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                let mut event_count = 0;
                // The hardware keeps a queue of events. We should ideally process all event from the queue before rendering
                // or we will get outdated event in the next frames. But move events are constantly added to the queue
                // so we would block the whole interface, so add an arbitrary threshold

                // 问题：一次触摸会出发很多次 
                while event_count < 15 && touch.data_available().unwrap() {
                    event_count += 1;
                    let button = slint::platform::PointerEventButton::Left;
                    let tp = touch.read_touch().unwrap();
                    if let Some(event) = tp
                        .map(|record| {
                            // 修改的地方：这里不能对 record.x 进行反向计算，否则 x 轴翻转。                            
                            let position = slint::PhysicalPosition::new(
                                (record.x as f32 * size.width as f32 / 320.) as _,
                                (record.y as f32 * size.height as f32 / 240.) as _,
                            )
                            .to_logical(window.scale_factor());
                            match last_touch.replace(position) {
                                Some(_) => WindowEvent::PointerMoved { position },
                                None => WindowEvent::PointerPressed { position, button },
                            }
                        })
                        .or_else(|| {                            
                            last_touch
                                .take()
                                .map(|position| WindowEvent::PointerReleased { position, button })
                        })
                    {
                        esp_println::println!("event = {:?}", event);
                        let is_pointer_release_event =
                            matches!(event, WindowEvent::PointerReleased { .. });

                        window.dispatch_event(event);

                        // removes hover state on widgets
                        if is_pointer_release_event {
                            window.dispatch_event(WindowEvent::PointerExited);
                        }
                    }
                }

                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                });
                if window.has_active_animations() {
                    continue;
                }
            }
        }
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        esp_println::println!("{}", arguments);
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}
impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ILI9341Rgb565, RST>>
// 或者 mipidsi::models::ILI9342CRgb565
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        //We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}
