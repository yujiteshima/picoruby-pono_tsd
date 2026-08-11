#!/usr/bin/env ruby
# frozen_string_literal: true

# Drive the TSD driver from host CRuby -- the same class the microcontroller
# runs, over a different transport. Point it at the simulator's pty, or at a
# real sensor on a TTL-USB adapter:
#
#   ruby sim/tsd_sim.rb pty --rpm 1150 --blades 3 &   # prints /dev/ttysNNN
#   ruby example/host_cruby.rb /dev/ttysNNN
#
#   stty -f /dev/cu.usbserial-XXXX 460800             # real sensor: set the
#   ruby example/host_cruby.rb /dev/cu.usbserial-XXXX # baud first
#
# No gems, no compilation: a Mac and the sensor are enough to watch frames.

require_relative "../mrblib/pono_tsd"

# The two methods the driver needs, over a plain IO.
class IoUart
  def initialize(path)
    @io = File.open(path, "r+")
    @io.binmode
    @io.sync = true
    begin
      require "io/console"
      @io.raw!
    rescue LoadError, Errno::ENOTTY, Errno::ENODEV
      # a pipe or a file works too; only real ttys need raw mode
    end
  end

  def read(len = nil)
    @io.read_nonblock(len || 4096)
  rescue IO::WaitReadable, EOFError
    nil
  end

  def write(data)
    @io.write(data)
  end
end

path = ARGV[0] or abort "usage: host_cruby.rb <device>"
tsd = TSD.new(uart: IoUart.new(path))

puts "version : #{tsd.software_version.inspect}"
puts "serial  : #{tsd.serial_number.inspect}"
puts "ranging : #{tsd.start_ranging.inspect}"

t0 = Time.now
samples = Array.new(256, 0.0)
held = tsd.fill(samples, timeout_ms: 15_000)
abort "fill timed out -- nothing on #{path}?" unless held
dt = Time.now - t0

puts "filled  : #{samples.size} samples in #{dt.round(2)} s " \
     "(#{(samples.size / dt).round(1)} Hz, held #{held}, dropped #{tsd.parser.dropped_bytes})"
puts "range   : #{samples.min.round(1)} .. #{samples.max.round(1)} mm"
puts "HOST CRUBY OK"
