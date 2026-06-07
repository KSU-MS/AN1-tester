use embassy_rp::{i2c::{Blocking, I2c}, peripherals::I2C0};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use mlx9064x::{MelexisCamera, Mlx90640Driver, mlx90640::Mlx90640};

#[embassy_executor::task]
pub async fn tire_temp_task(mut cam: Mlx90640Driver<I2c<'static, I2C0, Blocking>>, temp: &'static Signal<CriticalSectionRawMutex, f32>) {
    let mut temperatures = [0f32; Mlx90640::HEIGHT * Mlx90640::WIDTH];
    // cam.set_frame_rate(mlx9064x::FrameRate::SixtyFour);
    loop {
        let _ = cam.generate_image_if_ready(&mut temperatures);
        
        // grab the center area of the tire.
        // find the average

        let width = Mlx90640::WIDTH;
        let height = Mlx90640::HEIGHT;

        let x_start = width / 4;
        let x_end = width - x_start;
        let y_start = height / 4;
        let y_end = height - y_start;

        let mut sum = 0.0f32;
        let mut count = 0usize;

        for y in y_start..y_end {
            for x in x_start..x_end {
                let t = temperatures[y * width + x];
                if t.is_finite() {
                    sum += t;
                    count += 1;
                }
            }
        }

        let val: f32 = if count > 0 { sum / count as f32 } else { 0.0 };
        temp.signal(val);
    }
}
