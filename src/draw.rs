use core::fmt::Write;

use arrayvec::ArrayString;
use defmt::info;
use display_interface::DisplayError;
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_5X8, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::*,
    text::Text,
};

#[derive(Default)]
struct ViewData {
    points: [Point; 10],
    frame_count: u16,
    fps: u16,
}

#[embassy_executor::task]
pub async fn lcd_task(lcd: &'static mut super::BufferedDriver) {
    Timer::after_millis(100).await;

    info!("Starting LCD task...");
    let mut vd = ViewData::default();
    let one_sec = embassy_time::Duration::from_secs(1);
    let mut next_sec = embassy_time::Instant::now() + one_sec;

    loop {
        if let Ok(pts) = super::CHANNEL.try_receive() {
            for (i, p) in pts.iter().enumerate() {
                vd.points[i].x = 20 + 20 * i as i32;
                vd.points[i].y = 120 + *p;
            }
        }

        vd.frame_count += 1;
        if embassy_time::Instant::now() > next_sec {
            vd.fps = vd.frame_count;
            vd.frame_count = 0;
            next_sec = embassy_time::Instant::now() + one_sec;
        }

        draw(lcd, &vd).unwrap();
    }
}

const GREEN_LINE: PrimitiveStyle<Rgb565> = PrimitiveStyle::with_stroke(Rgb565::GREEN, 3);
const RED_TEXT: MonoTextStyle<Rgb565> = MonoTextStyle::new(&FONT_5X8, Rgb565::RED);

fn draw(lcd: &mut super::BufferedDriver, vd: &ViewData) -> Result<(), DisplayError> {
    lcd.clear();

    Polyline::new(&vd.points)
        .into_styled(GREEN_LINE)
        .draw(lcd)?;

    let mut fps = ArrayString::<16>::from("FPS: ").unwrap();
    write!(&mut fps, "{}", vd.fps).unwrap();
    Text::new(&fps, Point::new(40, 40), RED_TEXT).draw(lcd)?;

    lcd.flush()
}
