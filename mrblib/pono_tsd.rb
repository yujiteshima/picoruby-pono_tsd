# PONO TSD series (TSD10 / TSD20 / TSD50) single-point dToF LiDAR.
#
# The sensor free-runs: once ranging, it emits one 4-byte frame per
# measurement over UART (460800 8N1 from the factory), so the constant-rate
# sampling a spectrum needs is the sensor's job, not this driver's. At the
# series' rates (200 Hz on TSD20, 500 Hz on TSD50) parsing in Ruby costs a
# handful of dispatches per frame -- it is per-sample loops in the kilohertz
# that have to leave Ruby, not this.
#
# Everything here is plain Ruby against the mruby VM; there is no ports/
# directory and no C.
class TSD
  # The sensor reports this instead of a distance when the target is out of
  # range. Left in a series it would dwarf the real signal, so #fill
  # substitutes the last valid reading and counts what it replaced.
  OUT_OF_RANGE = 50000

  # Measuring frequency -> divisor, per the manual: f = 10000 / (divisor + 1).
  # TSD20 accepts these six; TSD50 is fixed at 500 Hz and has no such command.
  FREQUENCY_DIVISORS = {
    200 => 49,
    100 => 99,
    50 => 199,
    20 => 499,
    10 => 999,
    1 => 9999
  }

  CMD_BAUDRATE  = 0x06
  CMD_RANGING   = 0x0A
  CMD_FREQUENCY = 0x0B
  CMD_SERIAL    = 0x0D
  CMD_VERSION   = 0x16
  CMD_TO_IIC    = 0x1F

  # PicoRuby provides Kernel#sleep_ms; CRuby has sleep; bare mruby may have
  # neither. Probed once at load so the wait paths work wherever the class
  # does -- with neither, timeout loops still terminate, they just spin.
  HAS_SLEEP_MS = begin
    sleep_ms(0)
    true
  rescue NoMethodError, NameError
    false
  end
  HAS_SLEEP = HAS_SLEEP_MS || begin
    sleep(0)
    true
  rescue NoMethodError, NameError
    false
  end

  # Byte-stream -> frames. Two frame shapes share the stream:
  #   data:     5C | dist_lo | dist_hi | ck            (fixed 4 bytes)
  #   response: 5A | cmd|0x80 | len | payload... | ck  (len+4 bytes)
  # ck = ~(sum of the bytes between header and ck) & 0xFF, both shapes.
  # On any checksum failure one byte is dropped and scanning restarts, so
  # the parser walks back into sync through garbage or a partial frame.
  class Parser
    HEADER_DATA = 0x5C
    HEADER_RESP = 0x5A
    # Longest documented payload is 4 bytes; anything claiming more is noise
    # wearing a header byte, and believing it would stall the scan.
    MAX_PAYLOAD = 16

    attr_reader :out_of_range_count, :dropped_bytes

    def initialize
      @bytes = []
      @distances = []
      @responses = []
      @out_of_range_count = 0
      @dropped_bytes = 0
    end

    def push(data)
      return self unless data
      i = 0
      n = data.bytesize
      while i < n
        @bytes << data.getbyte(i)
        i += 1
      end
      scan
      self
    end

    # Oldest undelivered distance in mm (OUT_OF_RANGE included), or nil.
    def pop_distance
      @distances.shift
    end

    # Oldest undelivered [cmd, payload_bytes], or nil.
    def pop_response
      @responses.shift
    end

    def pending
      @distances.size
    end

    private

    def scan
      while true
        b = @bytes
        size = b.size
        break if size < 4
        head = b[0]
        if head == HEADER_DATA
          lo = b[1]
          hi = b[2]
          if ((~(lo + hi)) & 0xFF) == b[3]
            d = lo | (hi << 8)
            @out_of_range_count += 1 if d == OUT_OF_RANGE
            @distances << d
            @bytes = b[4, size - 4]
            next
          end
          drop
        elsif head == HEADER_RESP
          len = b[2]
          if len > MAX_PAYLOAD
            drop
            next
          end
          total = len + 4
          break if size < total
          sum = 0
          i = 1
          last = total - 1
          while i < last
            sum += b[i]
            i += 1
          end
          if ((~sum) & 0xFF) == b[last]
            @responses << [b[1], b[3, len]]
            @bytes = b[total, size - total]
            next
          end
          drop
        else
          drop
        end
      end
    end

    def drop
      @bytes.shift
      @dropped_bytes += 1
    end
  end

  attr_reader :parser, :last_distance

  # Bring your own UART (anything with #read and #write). TSD.open builds
  # one; this form is what tests use to feed the driver canned bytes.
  def initialize(uart:)
    @uart = uart
    @parser = Parser.new
    @last_distance = nil
  end

  def self.open(unit:, txd_pin:, rxd_pin:, baudrate: 460800, rx_buffer_size: 1024)
    new(uart: UART.new(
      unit: unit,
      txd_pin: txd_pin,
      rxd_pin: rxd_pin,
      baudrate: baudrate,
      rx_buffer_size: rx_buffer_size
    ))
  end

  # Move whatever the UART has buffered into the parser. Non-blocking.
  def pump
    @parser.push(@uart.read)
    self
  end

  # One distance in mm. Out-of-range frames are skipped unless raw: true.
  # nil on timeout.
  def read_distance(timeout_ms: 100, raw: false)
    t = timeout_ms
    while true
      pump
      while (d = @parser.pop_distance)
        if d == OUT_OF_RANGE
          return d if raw
        else
          @last_distance = d
          return d
        end
      end
      return nil if t <= 0
      wait_ms 1
      t -= 1
    end
  end

  # Fill an indexable (DSP::Buffer, Array -- anything with #size and #[]=)
  # with consecutive samples in mm. Out-of-range frames repeat the last
  # valid distance: the spectrum wants a continuous series, and a 50000
  # spike would drown it. Returns how many samples were substituted, or
  # nil on timeout. 256 samples at 10 Hz take 25.6 s, hence the default.
  def fill(buf, timeout_ms: 30_000)
    n = buf.size
    i = 0
    held = 0
    t = timeout_ms
    while i < n
      d = @parser.pop_distance
      if d.nil?
        pump
        d = @parser.pop_distance
      end
      if d.nil?
        return nil if t <= 0
        wait_ms 1
        t -= 1
        next
      end
      if d == OUT_OF_RANGE
        held += 1
        # Nothing to hold yet: drop the frame rather than invent a value.
        next if @last_distance.nil?
        d = @last_distance
      else
        @last_distance = d
      end
      buf[i] = d.to_f
      i += 1
    end
    held
  end

  # Discard everything buffered so far, e.g. after a pause between windows.
  def drain
    pump
    while @parser.pop_distance; end
    while @parser.pop_response; end
    self
  end

  def start_ranging(timeout_ms: 200)
    !send_command(CMD_RANGING, [0x02, 0x00], timeout_ms: timeout_ms).nil?
  end

  def stop_ranging(timeout_ms: 200)
    !send_command(CMD_RANGING, [0x00, 0x00], timeout_ms: timeout_ms).nil?
  end

  def set_frequency(hz, timeout_ms: 200)
    div = FREQUENCY_DIVISORS[hz]
    unless div
      raise ArgumentError, "unsupported frequency #{hz} (supported: #{FREQUENCY_DIVISORS.keys.join('/')})"
    end
    !send_command(CMD_FREQUENCY, [div & 0xFF, (div >> 8) & 0xFF], timeout_ms: timeout_ms).nil?
  end

  def frequency=(hz)
    set_frequency(hz)
  end

  def serial_number(timeout_ms: 200)
    r = send_command(CMD_SERIAL, [CMD_SERIAL, CMD_SERIAL, CMD_SERIAL, CMD_SERIAL], timeout_ms: timeout_ms)
    return nil unless r && r.size >= 4
    r[0] | (r[1] << 8) | (r[2] << 16) | (r[3] << 24)
  end

  def software_version(timeout_ms: 200)
    r = send_command(CMD_VERSION, [CMD_VERSION, CMD_VERSION], timeout_ms: timeout_ms)
    return nil unless r && r.size >= 2
    "#{r[1]}.#{r[0]}"
  end

  # Send one command frame and wait for its response (cmd | 0x80), pumping
  # data frames aside meanwhile -- responses interleave with the ranging
  # stream. Returns the response payload (possibly []), or nil on timeout.
  def send_command(cmd, payload, timeout_ms: 200)
    @uart.write(build_command(cmd, payload))
    want = cmd | 0x80
    t = timeout_ms
    while true
      pump
      while (r = @parser.pop_response)
        return r[1] if r[0] == want
      end
      return nil if t <= 0
      wait_ms 1
      t -= 1
    end
  end

  def build_command(cmd, payload)
    sum = cmd + payload.size
    frame = "\x5A".dup
    frame << cmd.chr
    frame << payload.size.chr
    payload.each do |x|
      x &= 0xFF
      sum += x
      frame << x.chr
    end
    frame << ((~sum) & 0xFF).chr
    frame
  end

  private

  def wait_ms(ms)
    if HAS_SLEEP_MS
      sleep_ms ms
    elsif HAS_SLEEP
      sleep(ms * 0.001)
    end
    ms
  end
end
