# SoapyESPSDR

Native SoapySDR support for the ESPARGOS `esp-sdr` protocol-6 burst receiver.
The module exposes the nominal 16, 40, and 80 MS/s rates, 13–69 MHz RF
bandwidth, integer-MHz tuning, manual gain, and hardware AGC.  Captures are
CRC-checked `CAP16` bursts buffered for `readStream()`.  The firmware transport
has gaps between bursts, so this is useful for spectrum/waterfall display but
is not a continuous demodulation source.

## Arch Linux

```sh
sudo pacman -S --needed cmake make gcc soapysdr zlib
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build
SoapySDRUtil --find
SoapySDRUtil --probe="driver=espsdr,device=/dev/ttyACM0"
```

The install step places the module in SoapySDR's ABI-specific module
directory (for example `/usr/lib/SoapySDR/modules0.8`).

If access is denied, add the user to the serial-device group or install a udev
rule for Espressif VID `303a`, PID `1001`, then reconnect the board. In Gqrx,
select `driver=espsdr,device=/dev/ttyACM0` and choose a supported nominal rate.
