# Fan rpm over a TSD20 -- the PicoRubyKaigi demo loop.
#
# Wiring: TSD20 pin2->3V3, pin6->GND, pin3(TX)->GP1, pin4(RX)->GP0.
# Run on a Pico 2 W under R2P2 with picoruby-dsp and picoruby-pono_tsd
# in the build.
#
# Shaped for R2P2's heap: the reused mag buffer keeps the steady state
# allocation-free except for one Spectrum per pass, and the explicit
# GC.start collects it while the next window is still filling -- the
# allocation-outruns-GC failure is what killed the N=2048 measurement.

BLADES = 3
RATE   = 200
N      = 256

tsd = TSD.open(unit: :RP2040_UART0, txd_pin: 0, rxd_pin: 1)
puts "TSD S#{tsd.serial_number} V#{tsd.software_version}"
tsd.start_ranging

fft = DSP::FFT.new(N)
buf = DSP::Buffer.new(N)
mag = DSP::Buffer.new(N / 2)

loop do
  held = tsd.fill(buf)
  unless held
    puts "timeout -- is the sensor wired and powered?"
    next
  end
  spec = fft.forward!(buf.hann!)
  # min_bin 3: the series rides on the standoff distance, and hann spreads
  # that DC into bins 1 and 2. Bin 3 upward is signal.
  f = spec.peak_frequency(sample_rate: RATE.to_f, min_bin: 3, mag: mag)
  if f
    rpm = f / BLADES * 60.0
    puts "#{rpm.round(1)} rpm  (blade pass #{f.round(2)} Hz, held #{held}, dropped #{tsd.parser.dropped_bytes})"
  else
    puts "no peak"
  end
  GC.start
end
