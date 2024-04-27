import odroid_wiringpi as wpi
import time

print("hey what's up dick head")

# Initialize the odroid_wiringpi library
wpi.wiringPiSetup()

# Define the GPIO pin (Using WiringPi pin number)
pin = 3  # As an example, I'm using pin 12 for GPIOX.16, change it as needed

def pwmTest():
    # Set the pin to PWM OUTPUT mode
    wpi.pinMode(pin, wpi.PWM_OUTPUT)

    # Calculate the PWM value
    period = 20000  # Period in microseconds
    pulse_width = 1100  # Pulse width in microseconds
    duty_cycle = (pulse_width / period) * 1024  # Convert to range for odroid_wiringpi

    # # Set the PWM range (resolution)
    wpi.pwmSetRange(1024)

    # Set the PWM clock to match the period of 20ms
    # The default PWM frequency of the WiringPi library is 19.2 MHz / 19200kHz
    # pwmFrequency in Hz = 19.2e6 / (pwmClock * pwmRange)
    # To get a period of 20ms, you can adjust the clock divisor accordingly:
    # 20ms = 20000us = 1 / (frequency)
    # Calculate to achieve as close to 20ms as possible:
    pwm_clock = 192  # Adjust this value if needed to achieve the correct frequency
    wpi.pwmSetClock(pwm_clock)

    # # Set the duty cycle
    wpi.pwmWrite(pin, int(duty_cycle))

# print("PWM setup complete. GPIO pin", pin, "is outputting PWM signal.")


def digitalWriteTest():

    # Set the pin to OUTPUT mode
    wpi.pinMode(pin, wpi.OUTPUT)

    # Timing variables
    high_time = 0.001  # High time in seconds (2ms)
    low_time = 0.019   # Low time in seconds (18ms)

    try:
        while True:
            # Turn the pin high
            wpi.digitalWrite(pin, wpi.HIGH)
            print("high")
            # Wait for high_time
            time.sleep(high_time)
            # Turn the pin low
            wpi.digitalWrite(pin, wpi.LOW)
            print("low")
            # Wait for low_time
            time.sleep(low_time)

    except KeyboardInterrupt:
        # Turn off the pin on a Keyboard Interrupt (Ctrl+C)
        wpi.digitalWrite(pin, wpi.LOW)
        print("\nGPIO pin", pin, "turned off. Exiting cleanly.")


# digitalWriteTest()

pwmTest()

