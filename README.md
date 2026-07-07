## Anything iN 1.2.1 Universal Data Acquisition board
This repository holds the rust code for the AN1.2 daq board.

### Folder structure
```
AN1-tester
├── .cargo
│   └── config.toml
├── .gitignore
├── build.rs
├── Cargo.lock
├── Cargo.toml
├── memory.x
├── README.md
├── rust-toolchain.toml
└── src
    ├── main.rs
    ├── messages.rs
    └── tasks
        ├── generic_adc.rs
        ├── mod.rs
        ├── tire_temp.rs
        ├── vn_rs.rs
        └── wheel_speed.rs
```

`main.rs` holds the main event loop.

`messages.rs` contains friendly names for the CAN messages. These need to be changed based on which board the code is running on.

### Compilation Adjustments

There are several changes that may need to be made. Change these for your current compilation requirements.

`.cargo/config.toml` has configuration for the various runners. `probe-rs` debugs programs over SWD (preferred when debugging), while `picotool` will build and load the program to a pico via the built-in USB bootloader (doesn't require an SWD/DAPlink debugger). It turns out that `elf2uf2-rs` is no longer maintained so that option no longer works.

`Cargo.toml` is where all of the dependencies are listed. Both `mcp2418fd` and `ksu-rs-dbc` have lines referencing github, however they may be commented out and changed to reference local folders. The lines referencing github should be uncommented when compiling.

in `main.rs`, the primary logging system can either be `defmt` or `log`. `defmt` is logging over debug probes, while `log` is for using `embassy-usb-logger`. USB logging can also be disabled by commenting out lines 62 and 63 of `main.rs`

---
### Deploying code

**IMPORTANT:** `src/messages.rs` needs to have message names changed to reflect the can messages from the board being used.

1. install rust/cargo

2. install picotool

3. hold down the boot button while plugging the board into your computer (btn on left side if the usb cable is facing down)

4. open an1-tester in your terminal

5. Use the following command to upload the program:
```sh
cargo run --release
```
