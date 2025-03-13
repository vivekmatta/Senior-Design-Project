from machine import Pin, ADC, PWM
import time

# Define GPIOs
motor_pin = Pin(5, Pin.OUT)   # Motor control via transistor
led_pin = Pin(4, Pin.OUT)     # LED indicator
potentiometer = ADC(Pin(36))  # Potentiometer input (A0)
potentiometer.atten(ADC.ATTN_11DB)  # Full range (0-3.3V)

# Microphone Input (MAX9814)
mic = ADC(Pin(32))  # Mic output to ADC pin
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
vibration_active = False
last_stop_press = 0  # For debouncing

# Improved microphone threshold settings
buffer_size = 20  # Number of samples to keep for ambient noise
mic_buffer = []  # Buffer to store recent microphone readings
noise_margin = 3  # Multiplier for threshold
min_threshold = 1500  # Minimum threshold to prevent false triggers
max_silence_value = 200  # Maximum value considered as "silence"

# Peak detection variables
peak_detection_window = 5  # Check for peaks in last 5 samples
min_peak_value = 2000  # Minimum value to be considered a peak
last_detection_time = 0
cool_down_period = 3  # Seconds to wait after a detection
debug_mode = False  # Set to True for verbose output

def stop_button_handler(pin):
    """Interrupt handler for stop button"""
    global stop_vibration, last_stop_press
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_stop_press) > 300:  # 300ms debounce
        stop_vibration = True
        last_stop_press = current_time
        print("✓ Stop button pressed via interrupt")

# Attach interrupt to stop button
stop_button.irq(trigger=Pin.IRQ_FALLING, handler=stop_button_handler)

def get_ambient_noise_level():
    """Calculate the ambient noise level from the buffer"""
    if len(mic_buffer) < 5:
        return 500  # Default value if not enough samples
    
    # Filter out high values that might be speech
    quiet_samples = [x for x in mic_buffer if x < 1000]
    
    if not quiet_samples:
        # If no quiet samples, use the lowest 25% of all samples
        sorted_samples = sorted(mic_buffer)
        quiet_samples = sorted_samples[:len(sorted_samples)//4]
    
    if not quiet_samples:
        return 500  # Fallback if still no quiet samples
    
    # Average of quiet samples
    return sum(quiet_samples) / len(quiet_samples)

def is_peak_detected(value):
    """Check if the current value is a significant peak above ambient noise"""
    ambient = get_ambient_noise_level()
    threshold = max(ambient * noise_margin, min_threshold)
    
    # A peak must be significantly above ambient and above the minimum peak value
    is_peak = value > threshold and value > min_peak_value
    
    if is_peak:
        print(f"● SOUND DETECTED! [{value}] > [{threshold:.0f}]")
    elif debug_mode and value > threshold * 0.7:
        print(f"  Near threshold: {value} vs {threshold:.0f}")
    
    return is_peak

def vibrate():
    """ Runs the vibration motor with potentiometer control for 5 seconds or until stopped """
    global stop_vibration, last_detection_time
    
    # Update last detection time
    last_detection_time = time.time()

    print("\n▶ VIBRATION STARTED")
    print("------------------")
    stop_vibration = False  # Reset stop flag
    start_time = time.time()  # Track start time

    while not stop_vibration:
        # Check if stop button is pressed and released
        if stop_button.value() == 0:
            time.sleep(0.05)  # Small debounce delay
            if stop_button.value() == 1:  # Button released
                stop_vibration = True
                print("✓ Stop button released - stopping vibration")
                break  # Exit the loop

        # Stop automatically after 5 seconds
        if time.time() - start_time >= 5:
            print("✓ Auto-stopping after 5 seconds")
            break  # Exit the loop

        # Read potentiometer value to adjust vibration and LED brightness
        pot_value = potentiometer.read()  # Read potentiometer (0-4095)
        duty_cycle = int((pot_value / 4095) * 1023)  # Scale to PWM (0-1023)

        motor_pwm.duty(duty_cycle)  # Adjust motor strength
        led_pin.duty(duty_cycle)  # Adjust LED brightness based on motor strength

        time.sleep(0.01)  # Short delay for better responsiveness

    # Ensure motor and LED are turned off after stopping
    motor_pwm.duty(0)  # Turn off motor
    led_pin.duty(0)  # Turn off LED
    print("■ VIBRATION STOPPED")
    print("------------------\n")

def calibrate_mic(duration=5):
    """Calibrate the microphone by sampling ambient noise"""
    print("\n⚙ CALIBRATING MICROPHONE")
    print("----------------------")
    print(f"  Please remain quiet for {duration} seconds...")
    
    # Collect calibration samples
    calibration_samples = []
    start_time = time.time()
    
    while time.time() - start_time < duration:
        mic_value = mic.read()
        calibration_samples.append(mic_value)
        time.sleep(0.05)
    
    if not calibration_samples:
        print("  No samples collected, using default values")
        return
    
    # Initialize mic_buffer with these values
    global mic_buffer
    mic_buffer = calibration_samples[-buffer_size:] if len(calibration_samples) > buffer_size else calibration_samples
    
    # Calculate ambient noise level
    ambient = get_ambient_noise_level()
    threshold = max(ambient * noise_margin, min_threshold)
    
    print(f"✓ CALIBRATION COMPLETE")
    print(f"  • Ambient noise level: {ambient:.0f}")
    print(f"  • Detection threshold: {threshold:.0f}")
    print("----------------------\n")
    
    # Add a short delay after calibration
    time.sleep(1)

def main():
    global stop_vibration, mic_buffer, last_detection_time
    
    print("\n======================")
    print("  SOUND TRIGGER v1.0  ")
    print("======================")
    
    calibrate_mic(5)  # Longer calibration for better baseline
    
    print("● SYSTEM READY")
    print("  Make a loud noise or press the start button to trigger vibration")
    print("  Press the stop button to stop vibration")
    print("----------------------------------------")
    
    last_print_time = 0
    recent_values = []  # For peak detection
    
    while True:
        # Read microphone value
        mic_value = mic.read()
        
        # Add value to buffer (for ambient noise calculation)
        if mic_value < 1000:  # Only add low values to avoid skewing the average
            mic_buffer.append(mic_value)
            if len(mic_buffer) > buffer_size:
                mic_buffer.pop(0)
        
        # Add to recent values for peak detection
        recent_values.append(mic_value)
        if len(recent_values) > peak_detection_window:
            recent_values.pop(0)
        
        # Print values every second (avoid flooding serial)
        current_time = time.time()
        if current_time - last_print_time >= 2:  # Changed to 2 seconds for less cluttered output
            if debug_mode:
                ambient = get_ambient_noise_level()
                print(f"  Mic: {mic_value}, Threshold: {max(ambient * noise_margin, min_threshold):.0f}")
            last_print_time = current_time
        
        # Check for cool-down period
        if current_time - last_detection_time < cool_down_period:
            time.sleep(0.01)
            continue
            
        # Check for start button press
        if start_button.value() == 0:
            time.sleep(0.05)  # Small debounce
            if start_button.value() == 0:
                print("● START BUTTON PRESSED")
                vibrate()
                continue
        
        # Check for peak detection
        if is_peak_detected(mic_value):
            vibrate()
        
        time.sleep(0.01)  # Small delay to prevent excessive CPU usage

# Run the main function
if __name__ == "__main__":
    main()
