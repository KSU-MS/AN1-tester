use embassy_rp::gpio::Input;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Instant, Timer};

pub struct Config {
    pub button_is_pull_up: bool,
    pub debounce_input: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            button_is_pull_up: false,
            debounce_input: true,
        }
    }
}

#[embassy_executor::task]
pub async fn generic_button_task(
    mut pin: Input<'static>,
    reading: &'static Signal<CriticalSectionRawMutex, bool>,
    config: Config,
) {
    loop {
        // Wait for either edge.
        pin.wait_for_any_edge().await;

        // Make sure they pressed it for more than 20ms
        if config.debounce_input {
            Timer::after_millis(20).await;
        }

        // Get state
        let state = pin.is_high();

        // If high is supposed to be true
        if state == true && config.button_is_pull_up {
            reading.signal(true);
        }
        // If low is supposed to be true
        else if state == false && !config.button_is_pull_up {
            reading.signal(true);
        }
        // Other wise false
        else {
            reading.signal(false);
        }
    }
}
