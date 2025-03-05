from machine import Pin, ADC, PWM, UART
import time

# Define GPIOs
motor_pin = Pin(5, Pin.OUT)   # Motor control via transistor
led_pin = Pin(4, Pin.OUT)     # LED indicator
potentiometer = ADC(Pin(36))  # Potentiometer input (A0)
potentiometer.atten(ADC.ATTN_11DB)  # Full range (0-3.3V)

# Buttons
start_button = Pin(8, Pin.IN, Pin.PULL_UP)   # Start button with pull-up resistor
stop_button = Pin(15, Pin.IN, Pin.PULL_UP)   # Stop button with pull-up resistor

# UART Setup (Serial Communication with Mac)
uart = UART(0, baudrate=115200)  # Default USB serial

# PWM Setup for Motor
motor_pwm = PWM(motor_pin, freq=1000)  # 1 kHz PWM frequency

# Ensure motor and LED are OFF at startup
motor_pwm.duty(0)  # Ensure motor is OFF
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

    print("ESP32 Ready. Waiting for start command (button or Mac).")

    while True:
        # Check for start button press
        if start_button.value() == 0:
            time.sleep(0.1)  # Debounce
            if start_button.value() == 0:
                print("Wake word recognized! (Button Pressed)")
                vibrate()

        # Check for serial command from Mac
        if uart.any():
            command = uart.readline().decode().strip()
            if command == "start":
                print("Wake word recognized! (Mac Clap Detection)")
                vibrate()

# Run the main function
main()
