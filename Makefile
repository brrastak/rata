
flash: build
	cargo flash --release --chip STM32F103C8

build:
	cargo build --release
	cargo size --release
	
size:
	cargo size --release

rtt:
	cargo embed --release

bloat:
	cargo bloat --release --crates

hex:
	cargo objcopy --release -- -O ihex target/rata.hex
