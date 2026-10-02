# SoapyESPSDR

Native SoapySDR support for the ESPARGOS `esp-sdr` protocol-6 receiver.
The module uses the ESP32-S3 firmware's CRC-checked `IQS` stream mode, exposing
the actual continuous decimated rates 15.625, 31.25, 62.5, 125, 250, 312.5,
and 625 kS/s. The 625 kS/s mode uses the 40 MS/s ring and is close to the
native USB Serial/JTAG transport limit; lower rates are safer.

For a wide waterfall view, the optional burst mode exposes the firmware's
nominal 16, 40, and 80 MS/s `CAP16` snapshots:

```text
soapy=driver=espsdr,device=/dev/ttyACM0,mode=burst
```

Burst mode is intentionally discontinuous and is not suitable for reliable
demodulation. The default `mode=iqs` provides truthful continuous decimated IQ.
It also exposes 13–69 MHz RF bandwidth, integer-MHz tuning, manual gain, and
hardware AGC. The raw 16/40/80 MS/s `CAP16` burst protocol is not presented as
a continuous stream because the serial transport cannot carry those rates.

The firmware and protocol implementation are maintained in the
[ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr) project. This driver
uses its protocol-6 `IQS` command and frame format as the source of truth.

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
select the SoapySDR input with device string
`soapy=driver=espsdr,device=/dev/ttyACM0` and choose an actual supported rate,
such as `250000`.

## Firmware and protocol

The authoritative firmware and protocol implementation is maintained in the
[ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr) repository. This driver
targets protocol version 6 on an ESP32-S3-WROOM-1 using native USB
Serial/JTAG (`303a:1001`, normally `/dev/ttyACM0`).

At open, the driver queries:

```text
INFO
CAPS
LIMITS?
RANGE?
```

The firmware controls used by the driver are:

```text
FREQ 2437
BANDWIDTH 20
GAIN HARDWARE
GAIN MANUAL 50
GAIN?
```

`CAP16 <samples> <rate-code>` returns signed int8 I/Q pairs in a `DATA` frame
with a CRC32. IQS mode uses the firmware command form:

```text
IQS 0 64 8 6
```

The fields are duration, decimation, bits per I/Q component, and rate code.
The driver parses and CRC-checks the resulting `IQS1` binary frames.

## Modes and limitations

The default IQS mode exposes actual continuous decimated rates:

```text
15625, 31250, 62500, 125000, 250000, 312500, 625000 samples/second
```

`250000` is the recommended starting point. `625000` provides more visible
spectrum but is close to the practical USB Serial/JTAG transport limit.

Burst mode is selected with:

```text
soapy=driver=espsdr,device=/dev/ttyACM0,mode=burst
```

It exposes nominal `16 MS/s`, `40 MS/s`, and `80 MS/s` rates using repeated
`CAP16` snapshots. Those rates describe the RF capture clock, not a continuous
host stream. Burst mode is useful for a wide waterfall but contains unavoidable
gaps and should not be used for reliable demodulation.

Raw continuous 16 MS/s int8 IQ would require approximately 32 MB/s, while a
2 Mbit/s UART can carry only about 0.2 MB/s before protocol overhead. Native
USB Serial/JTAG is faster but still cannot sustain the full raw 16/40/80 MS/s
payloads reliably.

The receiver is RX-only with one channel. Frequency requests are rounded to
the nearest firmware-supported whole MHz. The firmware advertises 100–6000 MHz
in software, but the ESP32-S3 RF frontend is practically useful mainly around
2.4 GHz. RF bandwidth is 13–69 MHz. Gain values 0–82 are PHY indices, not
calibrated dB values; both hardware AGC and manual gain are supported.

## Gqrx setup

For truthful continuous IQ mode, select the SoapySDR input and use:

```text
Device string: soapy=driver=espsdr,device=/dev/ttyACM0
Input rate:    250000
```

For a wide but discontinuous burst waterfall, use:

```text
Device string: soapy=driver=espsdr,device=/dev/ttyACM0,mode=burst
Input rate:    16000000
```

Gqrx may initially request 96 kS/s and print a warning that it is overriding
the rate with 250 kS/s or another supported rate. This is expected. The IQ
input rate is separate from the audio output rate used for demodulated sound.

Frequency, bandwidth, gain, pause, and resume operations stop and restart the
IQS stream so text control responses are not mixed with binary IQ frames.

## Troubleshooting

If `SoapySDR::Device::make() no match` appears, run `SoapySDRUtil --info` and
check that the module is installed under the ABI-specific directory, such as:

```text
/usr/lib/SoapySDR/modules0.8/libsoapy_espsdr.so
```

If the module is in a non-system prefix, set:

```sh
SOAPY_SDR_PLUGIN_PATH=/path/to/lib/SoapySDR/modules0.8 \
SoapySDRUtil --probe="driver=espsdr,device=/dev/ttyACM0"
```

If `/dev/ttyACM0` is missing, reconnect the board and check:

```sh
ls -l /dev/ttyACM*
```

Install a udev rule for Espressif VID `303a`, PID `1001` if permissions prevent
access. Use IQS mode at 250 kS/s or lower for stable continuous data; use burst
mode only when the wider waterfall is worth the gaps.
