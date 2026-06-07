use embassy_rp::gpio::Input;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Instant;

#[embassy_executor::task]
pub async fn wheel_speed_task(mut pin: Input<'static>, speed: &'static Signal<CriticalSectionRawMutex, u16>) {
    const THOUSANDTH_RADIANS_PER_TOOTH: u16 = 349;
    pin.wait_for_rising_edge().await;
    let mut prev = Instant::now();

    loop {
        pin.wait_for_rising_edge().await;
        let dt_us = Instant::now().checked_duration_since(prev).unwrap_or_default().as_micros();
        prev = Instant::now();

        if dt_us != 0 {
            // 349 thousandths of a radian * (1_000_000/dt_us)
            // divided by 10 since data logged is (radians/sec) * 100
            let s = u16::try_from(u64::from(THOUSANDTH_RADIANS_PER_TOOTH) * 100_000_u64 / dt_us);
            if s.is_ok() {
                speed.signal(s.unwrap());
            }
        }
    }
}