#!/usr/bin/python3

"""
Code associated with the Adafruit Snake Eyes Bonnet for Raspberry Pi.
PYTHON 3 ONLY. Provides convenience functions for reading and filtering
the board's ADC inputs, can run in background as a thread.
Requires adafruit-blinka (CircuitPython APIs for Python on big hardware)
and adafruit-circuitpython-ads1x15.
Does NOT handle button inputs or writing to displays -- other code handles
those tasks. Former is basic GPIO stuff, latter is done by the fbx2 code.
"""

import time
from threading import Thread
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class AdcChannel():
    """Corresponds to ONE CHANNEL of the Snake Eye Bonnet's ADS1015
       analog-to-digital converter. Provides clipping, optional inversion
       and noise filtering. Output range ('value' element) is always in
       range 0.0 to 1.0."""
    def __init__(self, channel):
        self.channel = channel # AnalogIn P0-P3
        self.enabled = False   # Disabled by default (until config() called)
        self.min_v = 0.0       # Min expected input voltage
        self.max_v = 3.3       # Max expected input voltage
        self.reverse = False   # If True, reverse output range (0.0 -> 1.0)
        self.filter = 0.0      # Noise reduction (0.0 <= n < 1.0)
        self.value = 0.5       # Initial state

    def config(self, **kwargs):
        """Reconfigure one channel of the Snake Eyes ADC. Accepts several
           keyword arguments that override default values/behaviors:
           min_v: Minimum voltage expected from ADC (e.g. 0.0)
           max_v: Maximum voltage expected from ADC (e.g. 3.3)
           reverse: If True, output range will be reversed.
           filter: Weighting applied to old vs new ADC reading. A value of
                   0.0 (the default) means no filtering will be applied.
                   Values approaching 1.0 make new readings slower on the
                   uptake (reducing minor noise) -- a value of 1.0 would
                   just make the original value stick permanently.
           Calling this function will make the corresponding ADC channel
           active (it will be polled in the SnakeEyesBonnet class run()
           function). There is no corresponding disable function."""
        self.enabled = True
        for key, value in kwargs.items():
            if key == "min_v":
                self.min_v = value
            elif key == "max_v":
                self.max_v = value
            elif key == "reverse":
                self.reverse = value
            elif key == "filter":
                self.filter = min(max(value, 0.0), 1.0)
        if self.min_v > self.max_v:
            self.min_v, self.max_v = self.max_v, self.min_v

    def read(self):
        """Poll ADC channel, applying scaling and filtering,
           store in 'value' member as well as return value."""
        if self.enabled:
            voltage = self.channel.voltage
            clipped = min(max(voltage, self.min_v), self.max_v)
            scaled = (clipped - self.min_v) / (self.max_v - self.min_v)
            if self.reverse:
                scaled = 1.0 - scaled
            self.value = (self.value * (self.filter) +
                          scaled * (1.0 - self.filter))
        return self.value

class SnakeEyesBonnet(Thread):
    """SnakeEyesBonnet encapsulates various analog-to-digital converter
       functionality of the Adafruit Snake Eyes Bonnet, providing up to
       four channels of analog input with clipping and filtering, with
       output ranging from 0.0 to 1.0 (rather than specific voltages or
       integer units)."""
    channel_dict = {
        0: ADS.P0,
        1: ADS.P1,
        2: ADS.P2,
        3: ADS.P3
    }

    def __init__(self, *args, **kwargs):
        """SnakeEyesBonnet constructor."""
        super(SnakeEyesBonnet, self).__init__(*args, **kwargs) # Thread
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1015(self.i2c)
        self.ads.gain = 1
        self.period = 1.0 / 60.0  # Polling inverval = 1/60 sec default
        self.print_values = False # Don't print values by default
        self.channel = []
        for index in range(4):
            self.channel.append(AdcChannel(
                                AnalogIn(self.ads, self.channel_dict[index])))

    def setup_channel(self, channel, **kwargs):
        """Configure one ADC channel of the Snake Eyes Bonnet. Pass channel
           number (0 to 3) as well as optional keyword arguments documented
           in AdcChannel.config()."""
        if 0 <= channel <= 3:
            self.channel[channel].config(**kwargs)

    def run(self):
        """Run in loop, polling active Snake Eyes Bonnet ADC channels and
           optionally printing results. Pass 'True' to print output, else
           it will run silently (updating the channel[].value member values).
           Default is False, so if invoked via threading it runs in the
           background. Polling interval is set using the SnakeEyesBonnet
           constructor with optional 'period' keyword argument.
           This function does not return. DO NOT rename this function,
           it's so named to work with Python 3's threading class."""
        while True:
            for channel in self.channel:
                if channel.enabled:
                    channel.read()
                    if self.print_values:
                        print("%.6f" % channel.value, end="  ")
            if self.print_values:
                print(flush=True)
            # Note that the 'period' value is not strictly speaking the
            # polling period, since there will be some overhead for the
            # code above to run, plus occasional garbage collection and
            # such. Period is really just the sleep time here...
            time.sleep(self.period)


# If script is invoked standalone, not imported by another script,
# run example test...
if __name__ == "__main__":
    BONNET = SnakeEyesBonnet()
    BONNET.period = 0.25        # Set up 1/4 second polling interval
    BONNET.print_values = True  # Display output as we go
    # Configure four ADC channels...
    BONNET.setup_channel(0, reverse=False, filter=0.0)
    BONNET.setup_channel(1, reverse=False, filter=0.0)
    BONNET.setup_channel(2, reverse=False, filter=0.0)
    BONNET.setup_channel(3, reverse=False, filter=0.0)
    # Run polling loop "manually" (not as thread)...
    BONNET.run()
