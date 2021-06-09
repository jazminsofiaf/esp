
# Rust on  ESP extensa architecture

this project is a Spike of running rust in ESP based on project https://github.com/MabezDev/xtensa-rust-quickstart

to build:
```
cargo xbuild
```

to flash project:
```
cargo espflash --chip esp32 --features="xtensa-lx-rt/lx6,xtensa-lx/lx6" /dev/tty.usbserial-0001
```

to see de serial port logs run the loupe in the arduino ide
![Arduino ide](https://github.com/jazminsofiaf/esp/tree/master/wiki_img/serial.png)

to configure navigate in the ide (Clion with rust plugin)
you should comment the target in .cargo/config

![config](https://github.com/jazminsofiaf/esp/tree/master/wiki_img/ide-config.png)

