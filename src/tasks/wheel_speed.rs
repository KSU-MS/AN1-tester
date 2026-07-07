use embassy_rp::gpio::Input;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant};

#[embassy_executor::task]
pub async fn wheel_speed_task(
    mut pin: Input<'static>,
    speed: &'static Signal<CriticalSectionRawMutex, (u16, f32)>,
) {
    // Number of bumps on the encoder ring, 19/rotation
    const TEETH_COUNT: u64 = 18;

    // (60 s/min) * (1 * 10^6 us/s) = 60000000 us/min
    const US_PER_MIN: u64 = 60_000_000;
    const US_PER_SEC: f32 = 1_000_000_f32;

    pin.wait_for_rising_edge().await;
    let mut prev_us = Instant::now();

    let mut prev_rpm = 0;

    loop {
        // Wait for new pulse, if it takes longer than 4 sec, assume RPM is 0
        match embassy_time::with_timeout(Duration::from_secs(4), pin.wait_for_rising_edge()).await {
            Ok(_) => {
                // Get the change in time
                let now = Instant::now();
                let dt_us = (now - prev_us).as_micros();
                prev_us = now; // Update for next cycle

                // (60000000 us/min) / ((now - prev_us) * (18 / rotation))
                let rpm = u16::try_from(US_PER_MIN / (dt_us * TEETH_COUNT));

                if let Ok(rpm) = rpm {
                    let delta_rpm = (rpm as i32 - prev_rpm) as f32 * US_PER_SEC / (dt_us as f32);

                    speed.signal((rpm, delta_rpm));

                    prev_rpm = rpm as i32;
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
