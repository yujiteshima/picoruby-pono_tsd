# Frame-level tests for TSD::Parser and the command builder, plus the driver
# running over a real UART object where the build provides one (the POSIX
# port backs UART with a ring buffer and an inject_rx test hook).
#
#   ../picoruby/bin/picoruby test/parser_test.rb
#
# Every byte vector that has a counterpart in the TSD20/TSD50 user manual is
# taken from the manual verbatim, so a failure here means we disagree with
# the datasheet, not with ourselves.

$failures = 0

def assert_equal(expected, actual, label)
  if expected == actual
    puts "  PASS #{label}"
  else
    $failures += 1
    puts "  FAIL #{label}: expected #{expected.inspect}, got #{actual.inspect}"
  end
end

def assert(cond, label)
  assert_equal(true, !!cond, label)
end

puts "== data frames =="

# Manual's own example: 5C 02 11 EC is 4354 mm.
p1 = TSD::Parser.new
p1.push("\x5C\x02\x11\xEC")
assert_equal 4354, p1.pop_distance, "manual example frame -> 4354 mm"
assert_equal nil, p1.pop_distance, "nothing left after one frame"

# Same frame delivered one byte at a time (UART reads split anywhere).
p2 = TSD::Parser.new
"\x5C\x02\x11\xEC".bytesize.times do |i|
  p2.push("\x5C\x02\x11\xEC".byteslice(i, 1))
end
assert_equal 4354, p2.pop_distance, "byte-at-a-time delivery"

# Out-of-range: 50000 = 0xC350 -> 5C 50 C3 EC.
p3 = TSD::Parser.new
p3.push("\x5C\x50\xC3\xEC")
assert_equal 50000, p3.pop_distance, "out-of-range frame delivers 50000"
assert_equal 1, p3.out_of_range_count, "out-of-range counted"

puts "== resync =="

# Garbage before a frame, including a stray 0x5C that starts a bogus frame.
p4 = TSD::Parser.new
p4.push("\xFF\x13\x5C" + "\x5C\x02\x11\xEC")
assert_equal 4354, p4.pop_distance, "recovers through leading garbage"
assert p4.dropped_bytes >= 3, "dropped the garbage bytes"

# A corrupted frame (bad checksum) must not poison the next one.
p5 = TSD::Parser.new
p5.push("\x5C\x02\x11\x00" + "\x5C\x02\x11\xEC")
assert_equal 4354, p5.pop_distance, "bad checksum dropped, next frame parses"

# A response header claiming an absurd payload length is garbage, not a stall.
p6 = TSD::Parser.new
p6.push("\x5A\xFF\xFF" + "\x5C\x02\x11\xEC")
assert_equal 4354, p6.pop_distance, "oversized len claim does not stall the scan"

puts "== response frames =="

# Manual: version response 5A 96 02 03 02 62 -> V2.3.
p7 = TSD::Parser.new
p7.push("\x5A\x96\x02\x03\x02\x62")
assert_equal [0x96, [3, 2]], p7.pop_response, "manual version response"

# Responses interleave with the ranging stream.
p8 = TSD::Parser.new
p8.push("\x5C\x02\x11\xEC" + "\x5A\x8A\x02\x02\x00\x71" + "\x5C\x50\xC3\xEC")
assert_equal 4354, p8.pop_distance, "distance before interleaved response"
assert_equal [0x8A, [2, 0]], p8.pop_response, "manual start-ranging ack"
assert_equal 50000, p8.pop_distance, "distance after interleaved response"

puts "== command builder (bytes per the manual) =="

t = TSD.new(uart: nil)
assert_equal "\x5A\x0A\x02\x02\x00\xF1", t.build_command(0x0A, [0x02, 0x00]), "start ranging"
assert_equal "\x5A\x0A\x02\x00\x00\xF3", t.build_command(0x0A, [0x00, 0x00]), "stop ranging"
assert_equal "\x5A\x0B\x02\xE7\x03\x08", t.build_command(0x0B, [0xE7, 0x03]), "frequency 10 Hz"
assert_equal "\x5A\x16\x02\x16\x16\xBB", t.build_command(0x16, [0x16, 0x16]), "read version"
assert_equal "\x5A\x06\x02\x80\x04\x73", t.build_command(0x06, [0x80, 0x04]), "baud 115200"

begin
  TSD.new(uart: nil).set_frequency(123)
  assert false, "set_frequency(123) raises"
rescue ArgumentError
  assert true, "set_frequency(123) raises"
end

puts "== driver over a real UART (POSIX inject_rx) =="

uart = nil
begin
  uart = UART.new(unit: :sim0, txd_pin: 0, rxd_pin: 1, baudrate: 460800, rx_buffer_size: 1024)
  uart = nil unless uart.respond_to?(:inject_rx)
rescue => e
  puts "  SKIP (no host UART here: #{e.class})"
  uart = nil
end

if uart
  tsd = TSD.new(uart: uart)

  # Distances flow through pump/read_distance; OOR is skipped by default.
  uart.inject_rx("\x5C\x50\xC3\xEC" + "\x5C\x02\x11\xEC")
  assert_equal 4354, tsd.read_distance(timeout_ms: 5), "read_distance skips out-of-range"
  assert_equal 4354, tsd.last_distance, "last_distance tracks the valid reading"

  # fill: leading OOR (nothing to hold) is dropped; later OOR holds.
  frame = "\x5C\x02\x11\xEC"
  oor = "\x5C\x50\xC3\xEC"
  uart.inject_rx(oor + frame + oor + frame + frame + frame)
  buf = [0.0, 0.0, 0.0, 0.0]
  held = tsd.fill(buf, timeout_ms: 5)
  assert_equal 2, held, "fill counts substituted and dropped OOR frames"
  assert_equal [4354.0, 4354.0, 4354.0, 4354.0], buf, "fill holds last valid over OOR"

  # send_command finds its ack even with data frames interleaved (write is
  # a no-op on the POSIX port, so the response is pre-injected).
  uart.inject_rx("\x5C\x02\x11\xEC" + "\x5A\x96\x02\x03\x02\x62")
  assert_equal "2.3", tsd.software_version(timeout_ms: 5), "software_version over the wire"
  assert_equal 4354, tsd.read_distance(timeout_ms: 5), "data frame survived the command wait"
end

puts
if $failures == 0
  puts "ALL OK"
else
  puts "FAILURES: #{$failures}"
end
