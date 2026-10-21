use embassy_rp::gpio::Input;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, TimeoutError};

#[embassy_executor::task(pool_size = 2)]
pub async fn wheel_speed_task(
    mut pin: Input<'static>,
    speed: &'static Signal<CriticalSectionRawMutex, (u16, f32)>,
) {
    const THOUSANDTH_RADIANS_PER_TOOTH: u64 = 349;
    // const US_PER_SEC: f32 = 1_000_000_f32;

    let mut prev_time = Instant::now();
    let mut prev_rpm: u16 = 0;

    loop {
        // Wait for new pulse, if it takes longer than 4 sec, assume RPM is 0
        match embassy_time::with_timeout(Duration::from_millis(500), pin.wait_for_rising_edge()).await {
            Ok(_) => {
                let dt_us = Instant::now()
                    .checked_duration_since(prev_time)
                    .unwrap_or_default()
                    .as_micros();

                // if there's less than 500µs: it is reading the same tooth a second time due to vibration
                // 500µs = 6665.202 RPM
                if dt_us < 500 {
                    continue;
                }

                prev_time = Instant::now();

                // dt = 0 means unwrap_or_default() returned the default value.
                if dt_us == 0 {
                    speed.signal((9_000, 0_f32));
                    continue;
                }

                // 349/1000 radians * (1_000_000/dt_us)
                // = 349 * 1_000/dt_us (radians/sec)
                // multiply by ≈9.549 for rpm

                let rpm = u16::try_from(THOUSANDTH_RADIANS_PER_TOOTH * 9_549_u64 / dt_us);

                if let Ok(rpm) = rpm {
                    let rpm_delta = rpm as f32 - prev_rpm as f32;
                    speed.signal((rpm, rpm_delta));
                    prev_rpm = rpm
                } else {
                    speed.signal((9_000, 0_f32));
                }
            }

            Err(TimeoutError) => {
                // if the signal has timed out.
                speed.signal((0, -1_f32 * prev_rpm as f32));
                prev_rpm = 0;
            }
        }
    }
}
