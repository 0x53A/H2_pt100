The esp32-c3 is based on the RISC-V architecture, which is fully integrated into LLVM/Rust mainline.

See https://docs.espressif.com/projects/rust/book/installation/riscv.html


```
rustup toolchain install nightly --component rust-src
rustup target add riscv32imc-unknown-none-elf
```

Now just install ``espflash``:

```
cargo install espflash
```

With this you can build and flash the program using ``cargo run --release`` (debug is magnitudes slower than release).