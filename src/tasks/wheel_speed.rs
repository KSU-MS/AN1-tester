use embassy_rp::gpio::Input;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant};

#[embassy_executor::task]
pub async fn wheel_speed_task(
    mut pin: Input<'static>,
    speed: &'static Signal<CriticalSectionRawMutex, (u16, f32)>,
) {
    const THOUSANDTH_RADIANS_PER_TOOTH: u16 = 349;
    const US_PER_SEC: f32 = 1_000_000_f32;

    pin.wait_for_rising_edge().await;
    let mut prev_us = Instant::now();

    let mut prev_rpm = 0;

    loop {
        // Wait for new pulse, if it takes longer than 4 sec, assume RPM is 0
        match embassy_time::with_timeout(Duration::from_secs(4), pin.wait_for_rising_edge()).await {
            Ok(_) => {
                let dt_us = Instant::now().checked_duration_since(prev_us).unwrap_or_default().as_micros();
                prev_us = Instant::now();
                
                if dt_us != 0 {
                    // 349 thousandths of a radian * (1_000_000/dt_us)
                    // divided by 10 since data logged is (radians/sec) * 100
                    // this is so fucking chopped, multiply by ≈9.55 to convert to rpm
                    let rpm = u16::try_from(u64::from(THOUSANDTH_RADIANS_PER_TOOTH) * 95_493_000_u64 / dt_us);
                    if let Ok(rpm) = rpm {
                        let delta_rpm = (rpm as i32 - prev_rpm) as f32 * US_PER_SEC / (dt_us as f32);
                        prev_rpm = rpm as i32;
                        speed.signal((rpm, delta_rpm));
                    } else {
                        speed.signal((9_000, 0_f32));
                    }
                }
            }
            
            Err(_) => {
                let rpm = 0;
                
                // We know it has been 4 seconds-ish, so delta T should just be 4
                let delta_rpm = (rpm - prev_rpm) as f32 / 4_f32;

                speed.signal((rpm as u16, delta_rpm));

                prev_rpm = rpm as i32;
            }
        }
    }
}
