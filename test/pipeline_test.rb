# End-to-end: simulated fan -> UART ring buffer -> TSD driver -> DSP -> rpm.
#
# Expects a fixture (SIM_META / SIM_BYTES from sim/tsd_sim.rb) prepended, and
# a build that carries picoruby-dsp and picoruby-pono_tsd -- see run_host.sh.
#
# This is the PicoRubyKaigi demo pipeline with the sensor swapped for its
# simulator: if this passes, the only thing the hardware can still argue
# about is the wiring and the sensor's own integration time.

N = 256

$failures = 0

def assert(cond, label)
  if cond
    puts "  PASS #{label}"
  else
    $failures += 1
    puts "  FAIL #{label}"
  end
end

uart = UART.new(unit: :sim0, txd_pin: 0, rxd_pin: 1, baudrate: 460800, rx_buffer_size: 4096)
tsd = TSD.new(uart: uart)

# The whole recording fits in the ring buffer, so inject it in one go; the
# driver still sees it through the same read path the ISR feeds on a board.
injected = uart.inject_rx(SIM_BYTES)
assert injected == SIM_BYTES.bytesize, "fixture fits the RX ring (#{injected} bytes)"

buf = DSP::Buffer.new(N)
held = tsd.fill(buf, timeout_ms: 100)
assert !held.nil?, "fill completed (#{N} samples)"
assert held == SIM_META[:oor_frames] || held.nil?, "out-of-range frames all substituted (#{held})"
assert tsd.parser.dropped_bytes == SIM_META[:garbage_bytes], "garbage bytes all dropped (#{tsd.parser.dropped_bytes})"

spec = DSP::FFT.new(N).forward(buf.hann!)

# min_bin 3: the series rides on a ~300 mm offset, and the hann window
# spreads that DC into bins 1 and 2. Bin 3 upward is signal.
rate = SIM_META[:rate].to_f
f = spec.peak_frequency(sample_rate: rate, min_bin: 3)
expected = SIM_META[:rpm] / 60.0 * SIM_META[:blades]
bin_width = rate / N

puts
puts "  blade-pass expected #{expected} Hz, measured #{f ? f.round(3) : 'nil'} Hz (bin #{bin_width.round(3)} Hz)"
if f
  rpm = f / SIM_META[:blades] * 60.0
  puts "  fan #{SIM_META[:rpm].round} rpm, measured #{rpm.round(1)} rpm"
  assert (f - expected).abs < bin_width, "peak within one bin of the blade-pass frequency"
else
  assert false, "peak_frequency returned nil"
end

puts
if $failures == 0
  puts "ALL OK"
else
  puts "FAILURES: #{$failures}"
end
