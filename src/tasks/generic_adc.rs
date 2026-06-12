use embassy_rp::adc::{Adc, Async, Channel};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Instant, Timer};

#[embassy_executor::task]
pub async fn generic_adc_task(
    mut adc_fella: Adc<'static, Async>,
    mut pin: Channel<'static>,
    reading: &'static Signal<CriticalSectionRawMutex, (u16, f32)>,
) {
    //
    //// Filter things
    let mut filtered = 0_f32;

    // fs = 2.5 kHz
    // fc = 1000 Hz
    let dt = 1.0 / 2_500.0;
    let fc = 1000.0;

    let rc = 1.0 / (2.0 * core::f32::consts::PI * fc);
    let alpha = dt / (rc + dt);

    //
    //// Delta things
    const SEC_PER_US: f32 = 0.000001;

    let mut prev_us = Instant::now();
    let mut prev_sample = 0_f32;

    loop {
        if let Ok(data) = adc_fella.read(&mut pin).await {
            // Get the change in time
            let now = Instant::now();
            let dt_us = (now - prev_us).as_micros();
            prev_us = now; // Update for next cycle

            // 1-pole low-pass
            filtered += alpha * (data as f32 - filtered);

            // Update the delta
            let delta_filter = (filtered - prev_sample) / (dt_us as f32 * SEC_PER_US);
            prev_sample = filtered;

            reading.signal((filtered as u16, delta_filter));
        }

        Timer::after_micros(400).await;
    }
}
