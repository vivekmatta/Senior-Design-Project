from machine import Pin, ADC, PWM
import time

# Define GPIOs
motor_pin = Pin(5, Pin.OUT)   # Motor control via transistor
led_pin = Pin(4, Pin.OUT)     # LED indicator
potentiometer = ADC(Pin(36))  # Potentiometer input (A0)
potentiometer.atten(ADC.ATTN_11DB)  # Full range (0-3.3V)

# Buttons
start_button = Pin(8, Pin.IN, Pin.PULL_UP)   # Start button with pull-up resistor
stop_button = Pin(15, Pin.IN, Pin.PULL_UP)   # Stop button with pull-up resistor

# PWM Setup for Motor
motor_pwm = PWM(motor_pin, freq=1000)  # 1 kHz PWM frequency

# Ensure motor and LED are OFF at startup
motor_pwm.duty(0)  # Make sure the motor is OFF
led_pin.off()  # Ensure LED is OFF

# Flags
stop_vibration = False
vibration_active = False  # Ensures vibration doesn't start automatically

def stop_button_pressed(pin):
    """ Interrupt handler for stop button press """
    global stop_vibration
    stop_vibration = True
    print("Acknowledgment button pressed")

# Attach interrupt to stop button (triggers on press)
stop_button.irq(trigger=Pin.IRQ_FALLING, handler=stop_button_pressed)

def vibrate_with_button():
    """ Wait for start button press, then run vibration until stop button is pressed """
    global stop_vibration, vibration_active

    print("System ready. Press the start button to begin vibration.")

    # Wait for start button press
    while True:
        if start_button.value() == 0:  # Start button pressed
            time.sleep(0.1)  # Debounce
            if start_button.value() == 0:  # Confirm press
                print("Wake word recognized!")
                break  # Exit loop and start vibration

    stop_vibration = False  # Reset stop flag
    vibration_active = True  # Mark vibration as active

    # Start vibration
    while vibration_active:
        if stop_vibration:  # Stop immediately if stop button is pressed
            break

        pot_value = potentiometer.read()  # Read potentiometer (0-4095)
        duty_cycle = int((pot_value / 4095) * 1023)  # Scale to PWM (0-1023)

        motor_pwm.duty(duty_cycle)  # Adjust motor strength
        led_pin.value(duty_cycle > 512)  # Turn LED on if vibration is strong

        print(f"Potentiometer: {pot_value}, PWM Duty: {duty_cycle}")
        time.sleep(0.1)

    # Turn off vibration when stop button is pressed
    motor_pwm.duty(0)
    led_pin.off()
    print("Vibration stopped")

# Ensure motor is off before running the program
motor_pwm.duty(0)  # Redundant safeguard

# Start program and wait for button presses
vibrate_with_button()


