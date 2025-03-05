from machine import Pin, ADC, PWM
import time

# Define GPIOs
motor_pin = Pin(5, Pin.OUT)   # Motor control via transistor
led_pin = Pin(4, Pin.OUT)     # LED indicator
potentiometer = ADC(Pin(36))  # Potentiometer input (A0)
potentiometer.atten(ADC.ATTN_11DB)  # Full range (0-3.3V)

# Microphone Input (MAX9814)
mic = ADC(Pin(32))  # Mic output to ADC pin (Use GPIO 34, or try 32/33)
mic.atten(ADC.ATTN_11DB)  # Full range (0-3.3V)

# Buttons
start_button = Pin(8, Pin.IN, Pin.PULL_UP)   # Start button with pull-up resistor
stop_button = Pin(15, Pin.IN, Pin.PULL_UP)   # Stop button with pull-up resistor

# PWM Setup for Motor
motor_pwm = PWM(motor_pin, freq=1000)  # 1 kHz PWM frequency

# Ensure motor and LED are OFF at startup
motor_pwm.duty(0)  # Ensure motor is OFF
led_pin.off()  # Ensure LED is OFF

# Flags
stop_vibration = False
vibration_active = False  # Ensures vibration doesn't start automatically

# Microphone Noise Threshold
MIC_THRESHOLD = 2000  # Adjust this based on noise levels (0-4095)

def stop_button_pressed(pin):
    """ Interrupt handler for stop button press """
    global stop_vibration
    stop_vibration = True
    print("Acknowledgment button pressed")

# Attach interrupt to stop button (triggers on press)
stop_button.irq(trigger=Pin.IRQ_FALLING, handler=stop_button_pressed)

def vibrate():
    """ Runs the vibration motor with potentiometer control until stopped """
    global stop_vibration

    print("Vibration started!")
    stop_vibration = False  # Reset stop flag

    while not stop_vibration:  # Run until stop button is pressed
        pot_value = potentiometer.read()  # Read potentiometer (0-4095)
        duty_cycle = int((pot_value / 4095) * 1023)  # Scale to PWM (0-1023)

        motor_pwm.duty(duty_cycle)  # Adjust motor strength
        led_pin.value(duty_cycle > 512)  # Turn LED on if vibration is strong

        print(f"Potentiometer: {pot_value}, PWM Duty: {duty_cycle}")
        time.sleep(0.1)

    motor_pwm.duty(0)  # Turn off motor
    led_pin.off()
    print("Vibration stopped")

def main():
    global stop_vibration, vibration_active

    print("ESP32 Ready. Waiting for start command (button or sound).")

    while True:
        # Check for start button press
        if start_button.value() == 0:
            time.sleep(0.1)  # Debounce
            if start_button.value() == 0:
                print("Wake word recognized! (Button Pressed)")
                vibrate()

        # Check microphone for loud sound
        mic_value = mic.read()  # Read microphone ADC value
        if mic_value > MIC_THRESHOLD:  # If sound is above threshold
            print(f"Sound Detected! (ADC Value: {mic_value}) - Triggering Vibration")
            vibrate()

        time.sleep(0.1)  # Small delay to prevent excessive CPU usage

# Run the main function
main()
