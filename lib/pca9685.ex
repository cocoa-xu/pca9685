defmodule Cirlute.PCA9685 do
  @moduledoc """
  PWM board PCA9685
  """

  use Bitwise

  alias Circuits.I2C
  alias Cirlute.PCA9685, as: T

  @behaviour Cirlute.PWM

  @doc """
  - address: I2C address.
  - i2c_handle: Circuits.I2C handle.
  - frequency: PWM frequency.
  """
  @enforce_keys [:address, :i2c_handle, :frequency]
  defstruct address: 0x40, i2c_handle: nil, frequency: 60

  @type pca9685_channel() :: 0..15
  @type pca9685_register() ::
          0x00
          | 0x01
          | 0x02
          | 0x03
          | 0x04
          | 0x06
          | 0x07
          | 0x08
          | 0x09
          | 0x10
          | 0x80
          | 0xFA
          | 0xFB
          | 0xFC
          | 0xFD
          | 0xFE

  @doc """
  Offsets for registers on PCA9685
  """
  def led0_on_l, do: 0x06
  def led0_on_h, do: 0x07
  def led0_off_l, do: 0x08
  def led0_off_h, do: 0x09

  def all_led0_on_l, do: 0xFA
  def all_led0_on_h, do: 0xFB
  def all_led0_off_l, do: 0xFC
  def all_led0_off_h, do: 0xFD

  def mode1, do: 0x00
  def mode2, do: 0x01
  def subadr1, do: 0x02
  def subadr2, do: 0x03
  def subadr3, do: 0x04
  def prescale, do: 0xFE

  def restart, do: 0x80
  def sleep, do: 0x10
  def allcall, do: 0x01
  def invrt, do: 0x10
  def outdrv, do: 0x04

  @doc """
  Initialise PCA9685 @i2c-1

  ## Examples

      iex> Cirlute.PCA9685.new()
      {:ok, %PCA9685{...}}

  """
  @spec new(non_neg_integer(), pos_integer(), number()) :: {:ok, %T{}} | {:error, term()}
  def new(address, i2c_bus, freq)
      when is_integer(address) and address > 0 and is_integer(i2c_bus) and i2c_bus > 0 and
             is_number(freq) and freq > 0 do
    with {:ok, handle} <- I2C.open("i2c-" <> to_string(i2c_bus)),
         self <- %T{address: address, i2c_handle: handle, frequency: freq},
         # initialise PCA9685
         :ok <- write_all_value(self, 0, 0),
         :ok <- write_byte_data(self, mode2(), <<outdrv()>>),
         :ok <- write_byte_data(self, mode1(), <<allcall()>>),
         _ <- :timer.sleep(5),
         {:ok, new_mode1} = read_byte_data(self, mode1()),
         :ok <- write_byte_data(self, mode1(), <<new_mode1 &&& ~~~sleep()>>),
         _ <- :timer.sleep(5),
         {:ok, self} <- set_frequency(self, 60) do
      {:ok, self}
    else
      {:error, reason} -> {:error, reason}
    end
  end

  @doc """
  Set PWM frequency

  ## Parameters

      - self: instance of PCA9685
      - freq: requested frequency

  ## Examples

      iex> {:ok, pwm} = Cirlute.PCA9685.new()
      iex> Cirlute.PCA9685.set_frequency(pwm, 60)
      {:ok, %PCA9685{...}}

  """
  @spec set_frequency(%T{}, number()) :: {:ok, %T{}} | {:error, term()}
  def set_frequency(self = %T{}, freq) when is_integer(freq) and freq > 0 do
    # calculate prescale value
    prescale = trunc(floor(25_000_000.0 / 4096.0 / freq - 1.0 + 0.5))
    # get current parameter
    with {:ok, old_mode} <- read_byte_data(self, mode1()),
         # set flag
         new_mode <- (old_mode &&& 0x7F) ||| 0x10,
         # write back
         :ok <- write_byte_data(self, mode1(), new_mode),
         # set new prescale
         :ok <- write_byte_data(self, prescale(), prescale),
         # restore to old parameter
         :ok <- write_byte_data(self, mode1(), old_mode),
         # wait 5ms
         _ <- :timer.sleep(5),
         :ok <- write_byte_data(self, mode1(), old_mode ||| 0x80) do
      {:ok, self}
    else
      {:error, reason} -> {:error, reason}
    end
  end

  @spec set_pwm(term(), non_neg_integer(), non_neg_integer()) :: :ok | {:error, term()}
  def set_pwm(self = %T{}, channel, pwm_value) do
    write(self, channel, 0, pwm_value)
  end

  @doc """
  Set on and off value on specific channel

  ## Parameters

      - self: instance of PCA9685
      - channel: channel
      - on: refer to the PCA9685 datasheet
      - off: refer to the PCA9685 datasheet

  """
  @spec write(%T{}, pca9685_channel(), integer(), integer()) :: :ok | {:error, term()}
  def write(self = %T{}, channel, on, off)
      when is_integer(channel) and 0 <= channel and channel <= 15 do
    with :ok <- write_byte_data(self, led0_on_l() + 4 * channel, <<on &&& 0xFF>>),
         :ok <- write_byte_data(self, led0_on_h() + 4 * channel, <<on >>> 8>>),
         :ok <- write_byte_data(self, led0_off_l() + 4 * channel, <<off &&& 0xFF>>),
         :ok <- write_byte_data(self, led0_off_h() + 4 * channel, <<off >>> 8>>) do
      :ok
    else
      {:error, reason} -> {:error, reason}
    end
  end

  @doc """
  Write data to I2C with self.address

  ## Parameters

      - self: instance of PCA9685
      - reg: register to write to
      - value: value that writes to `reg`

  """
  @spec write_byte_data(%T{}, pca9685_register(), binary() | integer()) :: :ok | {:error, term()}
  def write_byte_data(self = %T{}, reg, value)
      when is_integer(reg) and is_binary(value) do
    # write I2C data
    I2C.write(self.i2c_handle, self.address, <<reg>> <> value, [{:retries, 2}])
  end

  def write_byte_data(self = %T{}, reg, value) when is_integer(reg) and is_integer(value) do
    # convert `value` to `binary()`
    write_byte_data(self, reg, <<value>>)
  end

  @doc """
  Set on and off value on all channel
  """
  @spec write_all_value(%T{}, integer(), integer()) :: :ok | {:error, term()}
  def write_all_value(self = %T{}, on, off) do
    with :ok <- write_byte_data(self, all_led0_on_l(), <<on &&& 0xFF>>),
         :ok <- write_byte_data(self, all_led0_on_h(), <<on >>> 8>>),
         :ok <- write_byte_data(self, all_led0_off_l(), <<off &&& 0xFF>>),
         :ok <- write_byte_data(self, all_led0_off_h(), <<off >>> 8>>) do
      :ok
    else
      {:error, reason} -> {:error, reason}
    end
  end

  @doc """
  Read data from I2C with self.address

  ## Examples

      iex> {:ok, pwm} = Cirlute.PCA9685.new()
      iex> Cirlute.PCA9685.read_byte_data(pwm, 0)
      {:ok, 1}

  """
  @spec read_byte_data(%T{}, pca9685_register()) :: {:ok, integer()} | {:error, term()}
  def read_byte_data(self = %T{}, reg) when is_integer(reg) do
    # read 1 byte from the specified address and register
    with {:ok, byte} <- I2C.read(self.i2c_handle, self.address, reg) do
      # decode it to unsigned
      {:ok, byte |> :binary.decode_unsigned()}
    else
      {:error, reason} -> {:error, reason}
    end
  end
end
