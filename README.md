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
├── rust-toolchain.toml
├── src
│   └── main.rs
└── target
```

`main.rs` holds the primary code

### compilation
there are several changes that may need to be made.

`.cargo/config.toml` has configuration for the various runners. `probe-rs` debugs programs over SWD (preferred when debugging), while `picotool` will build and load the program to a pico via the built-in USB bootloader (doesn't require an SWD/DAPlink debugger). It turns out that `elf2uf2-rs` is no longer maintained so that option no longer works.

`Cargo.toml` is where all of the dependencies are listed. Both `mcp2418fd` and `ksu-rs-dbc` have lines referencing github, however they may be commented out and changed to reference local folders. The lines referencing github should be uncommented when compiling.

in `main.rs`, the primary logging system can either be `defmt` or `log`. `defmt` is logging over debug probes, while `log` is for using `embassy-usb-logger`.

change these for your current compilation requirements. If you're trying to deploy to actually run on the vehicle, use the following command to upload the program:
```sh
cargo run --release
```
