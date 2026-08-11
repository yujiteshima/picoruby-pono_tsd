# picoruby-pono_tsd

A [PicoRuby](https://github.com/picoruby/picoruby) driver for the PONO TSD
series of single-point dToF LiDAR modules — TSD10 (10 m / 50 Hz), TSD20
(20 m / 200 Hz), TSD50 (50 m / 500 Hz). All three speak the same UART
protocol; one driver covers the series.

Pure Ruby, `mrblib/` only — no C, no `ports/`. The sensor free-runs at a
fixed measuring rate and streams one 4-byte frame per measurement, so the
constant-rate sampling that spectral analysis needs is the sensor's job.
What is left for the driver is framing, checksums, resync, and turning
out-of-range readings into something an FFT can sit on — a few dispatches
per frame at 200 Hz, which is comfortably Ruby's side of the line. (The
per-*sample* kilohertz loops live in
[picoruby-dsp](https://github.com/yujiteshima/picoruby-dsp), in C. Same
line, argued from the other side.)

## Usage

```ruby
tsd = TSD.open(unit: :RP2040_UART0, txd_pin: 0, rxd_pin: 1)  # 460800 8N1
tsd.start_ranging
tsd.software_version       #=> "2.3"

tsd.read_distance          #=> 431 (mm)
```

The demo this was built for — a desk fan's rpm, from distance alone:

```ruby
BLADES = 3
RATE   = 200                       # TSD20's native rate

buf = DSP::Buffer.new(256)
fft = DSP::FFT.new(256)
loop do
  held = tsd.fill(buf)             # 256 consecutive samples, 1.28 s
  spec = fft.forward(buf.hann!)
  f = spec.peak_frequency(sample_rate: RATE.to_f, min_bin: 3)
  puts "#{(f / BLADES * 60).round(1)} rpm" if f
end
```

`min_bin: 3` is not decoration: the series rides on the fan's standoff
distance (a large DC term), and a hann window spreads DC into bins 1 and 2.
Bin 3 upward is signal.

## API

| Method | What it does |
|---|---|
| `TSD.open(unit:, txd_pin:, rxd_pin:, baudrate: 460800, rx_buffer_size: 1024)` | build on a fresh `UART` |
| `TSD.new(uart:)` | bring your own UART (or anything with `#read`/`#write`) |
| `#read_distance(timeout_ms: 100, raw: false)` | one distance in mm; skips out-of-range unless `raw:` |
| `#fill(buf, timeout_ms: 30_000)` | fill any `#size`/`#[]=` object with consecutive samples; returns substituted count |
| `#start_ranging` / `#stop_ranging` | returns true on ack |
| `#set_frequency(hz)` / `#frequency=` | 200/100/50/20/10/1 (TSD20; TSD50 is fixed at 500) |
| `#serial_number` / `#software_version` | identity, handy as an is-it-alive check |
| `#drain` | discard buffered frames, e.g. after a pause between windows |
| `#parser` | the `TSD::Parser` underneath, with `out_of_range_count` / `dropped_bytes` |

**Out-of-range readings**: past its range the sensor reports the sentinel
`50000`, not an error. Left in a series it becomes a spike that drowns the
spectrum, so `fill` repeats the last valid reading instead and returns how
often it had to. Leading out-of-range frames (nothing to hold yet) are
dropped, not invented.

## Protocol, as verified against the manual

Data frames are 4 bytes, responses are `len+4`; both checksum the bytes
between header and checksum with `~(sum) & 0xFF`:

```
data:      5C | dist_lo | dist_hi | ck        e.g. 5C 02 11 EC = 4354 mm
response:  5A | cmd|80  | len | payload | ck
command:   5A | cmd     | len | payload | ck
```

The test suite asserts the driver's bytes against every worked example in
the TSD20 user manual — start `5A 0A 02 02 00 F1`, stop `…F3`, 10 Hz
`5A 0B 02 E7 03 08`, the version handshake, the 4354 mm frame. One caveat
for anyone reading the manual: its serial-number example shows a checksum
of `5F` where the (otherwise five-for-five) formula gives `5D`; we take
that for a scan artifact in the PDF.

Frequency setting is `f = 10000 / (divisor + 1)` with the divisor
little-endian in the payload.

## Wiring (TSD20 ↔ Pico 2 W)

3.3 V logic on both sides — no level shifting. ~40 mA average.

| TSD20 pin | Pico 2 W |
|---|---|
| 2 (3.3 V) | 3V3(OUT), pin 36 |
| 6 (GND) | any GND |
| 3 (TX) | GP1 = UART0 RX, pin 2 |
| 4 (RX) | GP0 = UART0 TX, pin 1 |

Pins 1 and 5 stay unconnected. The pigtail ends in tinned strand — solder
header pins on rather than trusting bare strand in a breadboard.

## The simulator

`sim/tsd_sim.rb` (host CRuby, stdlib only) is the sensor's half of the
wire: a fan model — baseline distance, a dip per blade crossing, gaussian
noise — plus the sensor's two failure modes, out-of-range frames and stray
bytes. Deterministic via its own LCG, so committed fixtures reproduce
anywhere.

```sh
# regenerate the committed test fixture
ruby sim/tsd_sim.rb fixture --rpm 1150 --blades 3 --rate 200 \
    --samples 300 --oor 0.01 --garbage 0.005 --seed 42 \
    --out test/fixture_fan.rb

# raw bytes to stdout
ruby sim/tsd_sim.rb stream --samples 1000 > fan.bin

# real-time pseudo-terminal that answers start/stop/frequency commands
ruby sim/tsd_sim.rb pty --rpm 1150 --blades 3
```

## Tests

```sh
PICORUBY=../picoruby/bin/picoruby test/run_host.sh
```

Needs a POSIX picoruby build carrying this gem (parser suite) and
picoruby-dsp (pipeline suite). The parser suite also runs the driver over a
real `UART` object — the POSIX port backs it with the same ring buffer the
ISR feeds on a board, filled through its `inject_rx` test hook. The
pipeline suite pushes the committed fixture through
UART → driver → `DSP` and requires the blade-pass peak to land within one
bin of the truth; with the stock fixture that is 1150 rpm in, 1150.9 rpm
out, through 3 out-of-range frames and 5 bytes of line noise.

## License

MIT
