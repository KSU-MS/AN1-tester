use embassy_rp::adc::{Adc, Async, Channel};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};

#[cfg(feature = "rear_unified")]
static N: usize=2;
#[cfg(not(feature = "rear_unified"))]
static N: usize=1;

#[embassy_executor::task]
pub async fn generic_adc_task(
    mut adc: Adc<'static, Async>,
    mut channels: [Channel<'static>; N],
    reading: [&'static Signal<CriticalSectionRawMutex, (u16, f32)>; N],
) {
    let mut ticker = Ticker::every(Duration::from_hz(2_500));
    let mut prev_data = [0_u16; N];

    loop {
        for i in 0..N {
            if let Ok(data) = adc.read(&mut channels[i]).await {
                reading[i].signal((data.clamp(0, 65535_u16), data as f32 - prev_data[i] as f32));
                prev_data[i] = data;
            }
        }

        ticker.next().await;
    }
}
