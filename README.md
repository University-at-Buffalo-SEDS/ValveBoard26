# ValveBoard26 firmware

ValveBoard26 targets the STM32G491 and controls fill-system valves while
sampling the board's ADC inputs. It exchanges commands, acknowledgements, and
telemetry over SEDSNet CAN-FD.

CMake fetches stable SEDSNet v4.0.18 and SEDS LaunchCore v1.0.0 releases; no
dependency submodules are required. LaunchCore creates the linker scripts from
`Bootloader/board_config.h` and packages factory, application, and `.seds` OTA
images according to that BSP layout.

```sh
./build.py build --release
./build.py flash --release --method stm32prog-cli
./build.py clean
./build.py test
./build.py test --all --release
```

The default flash workflow uses the combined factory image at `0x08000000`.
See `./build.py flash --help` for alternate programmers. The full tests add
release and OTA builds, STM32G491 flash/RAM and SEDSNet-pool checks, ADC2 and
valve peripheral models, fault injection, long-duration profiling, and an
end-to-end valve command whose execution ACK must return to GroundStation.

Network definitions live in `config/sedsnet.json`; simulated hardware and
limits live in `sim/board.json`.
