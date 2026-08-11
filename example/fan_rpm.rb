# Fan rpm over a TSD20 -- the PicoRubyKaigi demo loop.
#
# Wiring: TSD20 pin2->3V3, pin6->GND, pin3(TX)->GP1, pin4(RX)->GP0.
# Run on a Pico 2 W under R2P2 with picoruby-dsp and picoruby-pono_tsd
# in the build.

BLADES = 3
RATE   = 200
N      = 256

tsd = TSD.open(unit: :RP2040_UART0, txd_pin: 0, rxd_pin: 1)
puts "TSD S#{tsd.serial_number} V#{tsd.software_version}"
tsd.start_ranging

fft = DSP::FFT.new(N)
buf = DSP::Buffer.new(N)

loop do
  held = tsd.fill(buf)
  unless held
    puts "timeout -- is the sensor wired and powered?"
    next
  end
  spec = fft.forward(buf.hann!)
  f = spec.peak_frequency(sample_rate: RATE.to_f, min_bin: 3)
  if f
    rpm = f / BLADES * 60.0
    puts "#{rpm.round(1)} rpm  (blade pass #{f.round(2)} Hz, held #{held}, dropped #{tsd.parser.dropped_bytes})"
  else
    puts "no peak"
  end
end
