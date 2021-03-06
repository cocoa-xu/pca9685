defmodule Cirlute.PCA9685.MixProject do
  use Mix.Project

  def project do
    [
      app: :cirlute_pca9685,
      version: "0.1.0",
      elixir: "~> 1.11",
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  def application do
    []
  end

  defp deps do
    [
      {:circuits_i2c, "~> 0.1"},
      {:cirlute_pwm, "~> 0.1.0", github: "cocoa-xu/pwm"}
    ]
  end
end
