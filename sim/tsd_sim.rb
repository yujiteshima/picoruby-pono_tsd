#!/usr/bin/env ruby
# frozen_string_literal: true

# PONO TSD10/20/50 simulator -- the sensor's half of the wire, in CRuby.
#
# Models a desk fan seen by the LiDAR: a constant background with a dip each
# time a blade crosses the beam, gaussian noise on top, and the sensor's two
# failure modes mixed in (out-of-range frames reading 50000, and stray bytes
# corrupting the stream). Frames and commands follow the TSD user manual.
#
#   fixture  -- write a committed test fixture (Ruby source: SIM_META + SIM_BYTES)
#     ruby sim/tsd_sim.rb fixture --rpm 1150 --blades 3 --rate 200 \
#          --samples 300 --oor 0.01 --garbage 0.005 --seed 42 \
#          --out test/fixture_fan.rb
#
#   stream   -- raw bytes to stdout (pipe into anything)
#     ruby sim/tsd_sim.rb stream --samples 1000 > fan.bin
#
#   pty      -- serve a pseudo-terminal in real time; prints the device path.
#               Point a serial-reading program at it. Answers start/stop and
#               frequency commands like the sensor would.
#     ruby sim/tsd_sim.rb pty --rpm 1150 --blades 3
#
# Deterministic by design: noise comes from a local LCG, so the same seed
# gives the same bytes on any Ruby.

require "optparse"

module TSDSim
  OUT_OF_RANGE = 50000

  # LCG + Box-Muller. Random.new(seed) would tie fixtures to the Ruby
  # version's PRNG; this keeps committed fixtures reproducible anywhere.
  class Rng
    M = 2**31

    def initialize(seed)
      @s = seed % M
    end

    def uniform
      @s = (1_103_515_245 * @s + 12_345) % M
      @s.to_f / M
    end

    def gauss
      u1 = uniform
      u1 = 1e-9 if u1 <= 0.0
      Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math::PI * uniform)
    end

    def byte
      (uniform * 256).to_i & 0xFF
    end
  end

  class FanSignal
    def initialize(rpm:, blades:, base_mm:, dip_mm:, duty:, noise_mm:, rng:)
      @rpm = rpm
      @blades = blades
      @base = base_mm
      @dip = dip_mm
      @duty = duty
      @noise = noise_mm
      @rng = rng
    end

    # Distance in mm at time t seconds.
    def at(t)
      phase = (t * @blades * @rpm / 60.0) % 1.0
      d = phase < @duty ? @base - @dip : @base
      d + @rng.gauss * @noise
    end
  end

  module Protocol
    module_function

    def checksum(bytes)
      (~bytes.sum) & 0xFF
    end

    def data_frame(mm)
      mm = mm.round
      mm = 0 if mm < 0
      mm = 65_535 if mm > 65_535
      lo = mm & 0xFF
      hi = (mm >> 8) & 0xFF
      [0x5C, lo, hi, checksum([lo, hi])].pack("C*")
    end

    def response_frame(cmd, payload)
      body = [cmd | 0x80, payload.size] + payload
      ([0x5A] + body + [checksum(body)]).pack("C*")
    end
  end

  # Emits the ranging byte stream with corruption mixed in, and keeps count
  # of what it injected so fixtures can assert against the truth.
  class Emitter
    attr_reader :oor_frames, :garbage_bytes

    def initialize(signal:, rate:, oor_prob:, garbage_prob:, rng:)
      @signal = signal
      @rate = rate
      @oor_prob = oor_prob
      @garbage_prob = garbage_prob
      @rng = rng
      @k = 0
      @oor_frames = 0
      @garbage_bytes = 0
    end

    def next_bytes
      out = +""
      if @rng.uniform < @garbage_prob
        n = 1 + (@rng.uniform * 3).to_i
        n.times { out << @rng.byte.chr }
        @garbage_bytes += n
      end
      if @rng.uniform < @oor_prob
        @oor_frames += 1
        out << Protocol.data_frame(OUT_OF_RANGE)
      else
        out << Protocol.data_frame(@signal.at(@k.to_f / @rate))
      end
      @k += 1
      out
    end
  end

  # The command half: feed it what the host wrote, get [cmd, payload] pairs.
  # Same resync rule as the driver's parser, reduced to the 5A shape.
  class CommandReader
    def initialize
      @buf = []
    end

    def push(data)
      data.each_byte { |b| @buf << b }
      cmds = []
      while @buf.size >= 4
        if @buf[0] != 0x5A || @buf[2] > 16
          @buf.shift
          next
        end
        total = @buf[2] + 4
        break if @buf.size < total
        body = @buf[1, total - 2]
        if Protocol.checksum(body) == @buf[total - 1]
          cmds << [@buf[1], @buf[3, @buf[2]]]
          @buf = @buf[total..] || []
        else
          @buf.shift
        end
      end
      cmds
    end
  end

  class CLI
    DEFAULTS = {
      rpm: 1150.0, blades: 3, rate: 200.0,
      base: 300.0, dip: 60.0, duty: 0.35, noise: 3.0,
      oor: 0.0, garbage: 0.0, seed: 42, samples: 300, out: nil
    }.freeze

    def self.run(argv)
      mode = argv.shift
      o = DEFAULTS.dup
      OptionParser.new do |p|
        p.on("--rpm F", Float) { |v| o[:rpm] = v }
        p.on("--blades N", Integer) { |v| o[:blades] = v }
        p.on("--rate F", Float) { |v| o[:rate] = v }
        p.on("--base-mm F", Float) { |v| o[:base] = v }
        p.on("--dip-mm F", Float) { |v| o[:dip] = v }
        p.on("--duty F", Float) { |v| o[:duty] = v }
        p.on("--noise-mm F", Float) { |v| o[:noise] = v }
        p.on("--oor F", Float) { |v| o[:oor] = v }
        p.on("--garbage F", Float) { |v| o[:garbage] = v }
        p.on("--seed N", Integer) { |v| o[:seed] = v }
        p.on("--samples N", Integer) { |v| o[:samples] = v }
        p.on("--out PATH") { |v| o[:out] = v }
      end.parse!(argv)

      case mode
      when "fixture" then new(o).fixture
      when "stream" then new(o).stream
      when "pty" then new(o).pty
      else
        warn "usage: tsd_sim.rb {fixture|stream|pty} [options]"
        exit 2
      end
    end

    def initialize(o)
      @o = o
      rng = Rng.new(o[:seed])
      signal = FanSignal.new(
        rpm: o[:rpm], blades: o[:blades], base_mm: o[:base],
        dip_mm: o[:dip], duty: o[:duty], noise_mm: o[:noise], rng: rng
      )
      @emitter = Emitter.new(
        signal: signal, rate: o[:rate],
        oor_prob: o[:oor], garbage_prob: o[:garbage], rng: rng
      )
    end

    def generate
      bytes = +""
      @o[:samples].times { bytes << @emitter.next_bytes }
      bytes
    end

    def fixture
      abort "fixture mode needs --out" unless @o[:out]
      bytes = generate
      meta = {
        rpm: @o[:rpm], blades: @o[:blades], rate: @o[:rate],
        samples: @o[:samples], base_mm: @o[:base], dip_mm: @o[:dip],
        duty: @o[:duty], noise_mm: @o[:noise], seed: @o[:seed],
        oor_frames: @emitter.oor_frames, garbage_bytes: @emitter.garbage_bytes
      }
      lines = bytes.unpack("C*").each_slice(16).map do |chunk|
        '  "' + chunk.map { |b| format('\x%02X', b) }.join + '"'
      end
      src = +"# Generated by sim/tsd_sim.rb -- do not edit.\n"
      src << "#   #{regen_command}\n"
      src << "SIM_META = #{meta.inspect}\n"
      src << "SIM_BYTES =\n"
      src << lines.join(" \\\n")
      src << "\n"
      File.write(@o[:out], src)
      puts "#{@o[:out]}: #{bytes.bytesize} bytes, #{@o[:samples]} frames " \
           "(#{meta[:oor_frames]} out-of-range, #{meta[:garbage_bytes]} garbage bytes)"
    end

    def stream
      $stdout.binmode
      $stdout.write(generate)
    end

    def pty
      require "pty"
      require "io/console"
      master, slave = PTY.open
      # Frames are binary and full of 0x0A/0x0D; the default line discipline
      # would echo them back at us and rewrite them in both directions.
      slave.raw!
      puts slave.path
      $stdout.flush
      reader = CommandReader.new
      ranging = true
      rate = @o[:rate]
      # Pace against absolute time, not sleep(interval): the loop's own cost
      # would otherwise shave the rate (measured ~168 Hz out of 200).
      next_at = Time.now.to_f
      loop do
        begin
          data = master.read_nonblock(256)
          reader.push(data).each do |cmd, payload|
            case cmd
            when 0x0A
              ranging = payload[0] == 0x02
            when 0x0B
              div = payload[0] | (payload[1] << 8)
              rate = 10_000.0 / (div + 1)
            end
            master.write(Protocol.response_frame(cmd, payload))
          end
        rescue IO::WaitReadable, Errno::EAGAIN
          # nothing written to us; keep streaming
        end
        master.write(@emitter.next_bytes) if ranging
        next_at += 1.0 / rate
        delay = next_at - Time.now.to_f
        if delay > 0
          sleep delay
        else
          next_at = Time.now.to_f # fell behind; step, don't spiral
        end
      end
    rescue Errno::EIO
      # peer closed the pty
    end

    private

    def regen_command
      o = @o
      "ruby sim/tsd_sim.rb fixture --rpm #{o[:rpm]} --blades #{o[:blades]} " \
        "--rate #{o[:rate]} --samples #{o[:samples]} --oor #{o[:oor]} " \
        "--garbage #{o[:garbage]} --seed #{o[:seed]} --out #{o[:out]}"
    end
  end
end

TSDSim::CLI.run(ARGV) if $PROGRAM_NAME == __FILE__
