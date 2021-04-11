
# Rust on  ESP extensa architecture

this project is a Spike of running rust in ESP based on project https://github.com/MabezDev/xtensa-rust-quickstart


to run:
```
cargo espflash --chip esp32 --example esp32  --features="xtensa-lx-rt/lx6,xtensa-lx/lx6,esp32-hal" /dev/tty.usbserial-0001
```