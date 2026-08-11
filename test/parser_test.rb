# Picotest coverage for the frame parser and command builder -- the pieces
# with byte-for-byte answers in the TSD user manual. The full host suites
# (real UART object, simulator fixture through DSP) live in hosttest/,
# out of the picotest runner's reach, because they need a POSIX build and picoruby-dsp.
class PonoTSDParserTest < Picotest::Test
  def test_manual_data_frame
    p = TSD::Parser.new
    p.push("\x5C\x02\x11\xEC")
    assert_equal 4354, p.pop_distance
    assert_nil p.pop_distance
  end

  def test_out_of_range_frame
    p = TSD::Parser.new
    p.push("\x5C\x50\xC3\xEC")
    assert_equal 50000, p.pop_distance
    assert_equal 1, p.out_of_range_count
  end

  def test_split_delivery
    p = TSD::Parser.new
    "\x5C\x02\x11\xEC".bytesize.times do |i|
      p.push("\x5C\x02\x11\xEC".byteslice(i, 1))
    end
    assert_equal 4354, p.pop_distance
  end

  def test_resync_through_garbage
    p = TSD::Parser.new
    p.push("\xFF\x13\x5C" + "\x5C\x02\x11\xEC")
    assert_equal 4354, p.pop_distance
    assert p.dropped_bytes >= 3
  end

  def test_bad_checksum_recovery
    p = TSD::Parser.new
    p.push("\x5C\x02\x11\x00" + "\x5C\x02\x11\xEC")
    assert_equal 4354, p.pop_distance
  end

  def test_oversized_len_claim
    p = TSD::Parser.new
    p.push("\x5A\xFF\xFF" + "\x5C\x02\x11\xEC")
    assert_equal 4354, p.pop_distance
  end

  def test_manual_version_response
    p = TSD::Parser.new
    p.push("\x5A\x96\x02\x03\x02\x62")
    assert_equal [0x96, [3, 2]], p.pop_response
  end

  def test_interleaved_stream
    p = TSD::Parser.new
    p.push("\x5C\x02\x11\xEC" + "\x5A\x8A\x02\x02\x00\x71" + "\x5C\x50\xC3\xEC")
    assert_equal 4354, p.pop_distance
    assert_equal [0x8A, [2, 0]], p.pop_response
    assert_equal 50000, p.pop_distance
  end

  def test_command_builder_matches_manual
    t = TSD.new(uart: nil)
    assert_equal "\x5A\x0A\x02\x02\x00\xF1", t.build_command(0x0A, [0x02, 0x00])
    assert_equal "\x5A\x0A\x02\x00\x00\xF3", t.build_command(0x0A, [0x00, 0x00])
    assert_equal "\x5A\x0B\x02\xE7\x03\x08", t.build_command(0x0B, [0xE7, 0x03])
    assert_equal "\x5A\x16\x02\x16\x16\xBB", t.build_command(0x16, [0x16, 0x16])
    assert_equal "\x5A\x06\x02\x80\x04\x73", t.build_command(0x06, [0x80, 0x04])
  end

  def test_frequency_validation
    assert_raise(ArgumentError) do
      TSD.new(uart: nil).set_frequency(123)
    end
  end
end
