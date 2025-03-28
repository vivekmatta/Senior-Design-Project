/*
 * Sound Trigger v1.0
 * 
 * ESP-IDF implementation for ESP32-PICO-Mini-02
 * 
 * This code monitors sound levels via a microphone (MAX9814)
 * and triggers a vibration motor when sound peaks are detected.
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "driver/gpio.h"
 #include "driver/adc.h"
 #include "driver/ledc.h"
 #include "esp_log.h"
 #include "esp_adc_cal.h"
 #include "esp_timer.h"
 
 // Tag for ESP log
 static const char *TAG = "SOUND_TRIGGER";
 
 // Pin definitions for ESP32-PICO-Mini-02
 #define MOTOR_PIN           GPIO_NUM_5   // Motor control via transistor
 #define LED_PIN             GPIO_NUM_4   // LED indicator using PWM
 #define POTENTIOMETER_PIN   ADC1_CHANNEL_0  // Potentiometer input (ADC)
 #define MIC_PIN             ADC1_CHANNEL_4  // Microphone input
 #define START_BUTTON_PIN    GPIO_NUM_0   // Boot button can be used as start
 #define STOP_BUTTON_PIN     GPIO_NUM_15  // GPIO15 for stop button
 
 // PWM settings
 #define PWM_FREQ            1000    // 1 kHz PWM frequency
 #define PWM_RESOLUTION      LEDC_TIMER_10_BIT  // 10-bit resolution (0-1023)
 #define PWM_CHANNEL_MOTOR   LEDC_CHANNEL_0     // PWM channel for motor
 #define PWM_CHANNEL_LED     LEDC_CHANNEL_1     // PWM channel for LED
 #define PWM_TIMER           LEDC_TIMER_0       // Timer for PWM
 
 // Microphone threshold settings
 #define BUFFER_SIZE         20      // Number of samples for ambient noise
 #define NOISE_MARGIN        3.0     // Multiplier for threshold
 #define MIN_THRESHOLD       1500    // Minimum threshold to prevent false triggers
 #define MAX_SILENCE_VALUE   200     // Maximum value for "silence"
 #define PEAK_DETECTION_WINDOW 5     // Check for peaks in last 5 samples
 #define MIN_PEAK_VALUE      2000    // Minimum value to be considered a peak
 #define COOL_DOWN_PERIOD    3000    // Milliseconds to wait after detection
 #define DEBUG_MODE          false   // Set to true for verbose output
 
 // Global variables
 volatile bool stopVibration = false;
 bool vibrationActive = false;
 uint64_t lastStopPress = 0;  // For debouncing
 uint64_t lastDetectionTime = 0;
 uint64_t lastPrintTime = 0;
 
 // Buffers for sound processing
 int micBuffer[BUFFER_SIZE] = {0};
 int micBufferIndex = 0;
 int recentValues[PEAK_DETECTION_WINDOW] = {0};
 int recentValuesIndex = 0;
 
 // ADC Calibration
 esp_adc_cal_characteristics_t adc_chars;
 
 // Queue for handling button events
 QueueHandle_t button_event_queue;
 
 // Function prototypes
 void button_task(void *pvParameter);
 int getAmbientNoiseLevel();
 bool isPeakDetected(int value);
 void vibrate();
 void calibrateMic(int duration);
 void initGPIO();
 void initPWM();
 void initADC();
 
 // Button event structure
 typedef struct {
     uint32_t button_pin;
     uint64_t event_time;
 } button_event_t;
 
 // ISR for button events
 static void IRAM_ATTR gpio_isr_handler(void* arg) {
     uint32_t gpio_num = (uint32_t) arg;
     button_event_t event;
     event.button_pin = gpio_num;
     event.event_time = esp_timer_get_time() / 1000; // Convert to milliseconds
     xQueueSendFromISR(button_event_queue, &event, NULL);
 }
 
 void app_main() {
     // Initialize NVS
     ESP_LOGI(TAG, "Initializing system...");
     
     // Create queue for button events
     button_event_queue = xQueueCreate(10, sizeof(button_event_t));
     
     // Initialize components
     initGPIO();
     initPWM();
     initADC();
     
     // Start button handling task
     xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
     
     // Display welcome message
     ESP_LOGI(TAG, "\n======================");
     ESP_LOGI(TAG, "  SOUND TRIGGER v1.0  ");
     ESP_LOGI(TAG, "======================");
     
     // Calibrate microphone
     calibrateMic(5);
     
     ESP_LOGI(TAG, "● SYSTEM READY");
     ESP_LOGI(TAG, "  Make a loud noise or press the start button to trigger vibration");
     ESP_LOGI(TAG, "  Press the stop button to stop vibration");
     ESP_LOGI(TAG, "----------------------------------------");
     
     // Main loop
     while (1) {
         // Read microphone value
         int micValue = adc1_get_raw(MIC_PIN);
         
         // Add value to buffer (for ambient noise calculation)
         if (micValue < 1000) {  // Only add low values to avoid skewing the average
             micBuffer[micBufferIndex] = micValue;
             micBufferIndex = (micBufferIndex + 1) % BUFFER_SIZE;
         }
         
         // Add to recent values for peak detection
         recentValues[recentValuesIndex] = micValue;
         recentValuesIndex = (recentValuesIndex + 1) % PEAK_DETECTION_WINDOW;
         
         // Print values periodically
         uint64_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds
         if (currentTime - lastPrintTime >= 2000) {  // Every 2 seconds
             if (DEBUG_MODE) {
                 int ambient = getAmbientNoiseLevel();
                 int threshold = (ambient * NOISE_MARGIN > MIN_THRESHOLD) ? ambient * NOISE_MARGIN : MIN_THRESHOLD;
                 ESP_LOGI(TAG, "  Mic: %d, Threshold: %d", micValue, threshold);
             }
             lastPrintTime = currentTime;
         }
         
         // Check for cool-down period
         if (currentTime - lastDetectionTime < COOL_DOWN_PERIOD) {
             vTaskDelay(10 / portTICK_PERIOD_MS);
             continue;
         }
         
         // Manual check for start button press
         if (gpio_get_level(START_BUTTON_PIN) == 0) {
             vTaskDelay(50 / portTICK_PERIOD_MS);  // Small debounce
             if (gpio_get_level(START_BUTTON_PIN) == 0) {
                 ESP_LOGI(TAG, "● START BUTTON PRESSED");
                 vibrate();
                 continue;
             }
         }
         
         // Check for peak detection
         if (isPeakDetected(micValue)) {
             vibrate();
         }
         
         vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to prevent excessive CPU usage
     }
 }
 
 // Initialize GPIO pins
 void initGPIO() {
     // Configure button GPIOs
     gpio_config_t io_conf;
     
     // Config for input pins (buttons) with pull-up
     io_conf.intr_type = GPIO_INTR_NEGEDGE;
     io_conf.mode = GPIO_MODE_INPUT;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
     io_conf.pin_bit_mask = (1ULL << START_BUTTON_PIN) | (1ULL << STOP_BUTTON_PIN);
     gpio_config(&io_conf);
     
     // Install ISR service
     gpio_install_isr_service(0);
     
     // Hook ISRs to specific pins
     gpio_isr_handler_add(STOP_BUTTON_PIN, gpio_isr_handler, (void*) STOP_BUTTON_PIN);
     // Start button handled in main loop
     
     ESP_LOGI(TAG, "GPIO initialized");
 }
 
 // Initialize PWM outputs
 void initPWM() {
     // Motor PWM configuration
     ledc_timer_config_t motor_timer = {
         .duty_resolution = PWM_RESOLUTION,
         .freq_hz = PWM_FREQ,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .timer_num = PWM_TIMER
     };
     ledc_timer_config(&motor_timer);
     
     ledc_channel_config_t motor_channel = {
         .channel = PWM_CHANNEL_MOTOR,
         .duty = 0,
         .gpio_num = MOTOR_PIN,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .timer_sel = PWM_TIMER
     };
     ledc_channel_config(&motor_channel);
     
     // LED PWM configuration (using same timer)
     ledc_channel_config_t led_channel = {
         .channel = PWM_CHANNEL_LED,
         .duty = 0,
         .gpio_num = LED_PIN,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .timer_sel = PWM_TIMER
     };
     ledc_channel_config(&led_channel);
     
     // Ensure motor and LED are OFF at startup
     ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_MOTOR, 0);
     ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_MOTOR);
     ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_LED, 0);
     ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_LED);
     
     ESP_LOGI(TAG, "PWM initialized");
 }
 
 // Initialize ADC for microphone and potentiometer
 void initADC() {
     // ADC1 configuration
     adc1_config_width(ADC_WIDTH_BIT_12);
     adc1_config_channel_atten(MIC_PIN, ADC_ATTEN_DB_11);
     adc1_config_channel_atten(POTENTIOMETER_PIN, ADC_ATTEN_DB_11);
     
     // Characterize ADC
     esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
     
     ESP_LOGI(TAG, "ADC initialized");
 }
 
 // Button event handling task
 void button_task(void *pvParameter) {
     button_event_t event;
     
     while (1) {
         if (xQueueReceive(button_event_queue, &event, portMAX_DELAY)) {
             // Debounce
             uint64_t current_time = event.event_time;
             
             if (event.button_pin == STOP_BUTTON_PIN) {
                 if (current_time - lastStopPress > 300) {  // 300ms debounce
                     stopVibration = true;
                     lastStopPress = current_time;
                     ESP_LOGI(TAG, "✓ Stop button pressed via interrupt");
                 }
             }
         }
     }
 }
 
 // Calculate the ambient noise level from the buffer
 int getAmbientNoiseLevel() {
     // Check if we have enough samples
     int validSamples = 0;
     for (int i = 0; i < BUFFER_SIZE; i++) {
         if (micBuffer[i] > 0) validSamples++;
     }
     
     if (validSamples < 5) {
         return 500;  // Default value if not enough samples
     }
     
     // Count and sum the quiet samples
     int quietSamples[BUFFER_SIZE];
     int quietCount = 0;
     
     for (int i = 0; i < BUFFER_SIZE; i++) {
         if (micBuffer[i] < 1000 && micBuffer[i] > 0) {
             quietSamples[quietCount++] = micBuffer[i];
         }
     }
     
     if (quietCount == 0) {
         // If no quiet samples, sort and use the lowest 25%
         int sortedSamples[BUFFER_SIZE];
         int validCount = 0;
         
         // Copy valid values to sorted array
         for (int i = 0; i < BUFFER_SIZE; i++) {
             if (micBuffer[i] > 0) {
                 sortedSamples[validCount++] = micBuffer[i];
             }
         }
         
         // Simple bubble sort (for small arrays)
         for (int i = 0; i < validCount - 1; i++) {
             for (int j = 0; j < validCount - i - 1; j++) {
                 if (sortedSamples[j] > sortedSamples[j + 1]) {
                     // Swap
                     int temp = sortedSamples[j];
                     sortedSamples[j] = sortedSamples[j + 1];
                     sortedSamples[j + 1] = temp;
                 }
             }
         }
         
         // Use lowest 25%
         int samplesToUse = (validCount / 4 > 0) ? validCount / 4 : 1;
         int sum = 0;
         for (int i = 0; i < samplesToUse; i++) {
             sum += sortedSamples[i];
         }
         return sum / samplesToUse;
     }
     
     // Average of quiet samples
     int sum = 0;
     for (int i = 0; i < quietCount; i++) {
         sum += quietSamples[i];
     }
     return sum / quietCount;
 }
 
 // Check if the current value is a significant peak above ambient noise
 bool isPeakDetected(int value) {
     int ambient = getAmbientNoiseLevel();
     int threshold = (ambient * NOISE_MARGIN > MIN_THRESHOLD) ? ambient * NOISE_MARGIN : MIN_THRESHOLD;
     
     // A peak must be significantly above ambient and above the minimum peak value
     bool isPeak = value > threshold && value > MIN_PEAK_VALUE;
     
     if (isPeak) {
         ESP_LOGI(TAG, "● SOUND DETECTED! [%d] > [%d]", value, threshold);
     } else if (DEBUG_MODE && value > threshold * 0.7) {
         ESP_LOGI(TAG, "  Near threshold: %d vs %d", value, threshold);
     }
     
     return isPeak;
 }
 
 // Run the vibration motor with potentiometer control until stopped
 void vibrate() {
     // Update last detection time
     lastDetectionTime = esp_timer_get_time() / 1000;
     
     // Start vibration immediately
     ESP_LOGI(TAG, "\n▶ VIBRATION STARTED");
     ESP_LOGI(TAG, "------------------");
     stopVibration = false;  // Reset stop flag
     
     while (!stopVibration) {  // Run until stop flag is set
         // Manual check for stop button in addition to interrupt
         if (gpio_get_level(STOP_BUTTON_PIN) == 0) {
             vTaskDelay(50 / portTICK_PERIOD_MS);  // Small delay
             if (gpio_get_level(STOP_BUTTON_PIN) == 1) {  // Button released
                 stopVibration = true;
                 ESP_LOGI(TAG, "✓ Stop button released - stopping vibration");
             }
         }
         
         // Read potentiometer value to adjust vibration and LED brightness
         int potValue = adc1_get_raw(POTENTIOMETER_PIN);  // Read potentiometer (0-4095)
         int dutyCycle = (potValue * 1023) / 4095;  // Scale to PWM (0-1023)
         
         ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_MOTOR, dutyCycle);
         ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_MOTOR);
         
         ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_LED, dutyCycle);
         ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_LED);
         
         vTaskDelay(10 / portTICK_PERIOD_MS);  // Short delay for better responsiveness
     }
     
     // Ensure motor and LED are turned off
     ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_MOTOR, 0);
     ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_MOTOR);
     
     ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_LED, 0);
     ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_LED);
     
     ESP_LOGI(TAG, "■ VIBRATION STOPPED");
     ESP_LOGI(TAG, "------------------\n");
 }
 
 // Calibrate the microphone by sampling ambient noise
 void calibrateMic(int duration) {
     ESP_LOGI(TAG, "\n⚙ CALIBRATING MICROPHONE");
     ESP_LOGI(TAG, "----------------------");
     ESP_LOGI(TAG, "  Please remain quiet for %d seconds...", duration);
     
     // Reset buffer
     for (int i = 0; i < BUFFER_SIZE; i++) {
         micBuffer[i] = 0;
     }
     micBufferIndex = 0;
     
     // Collect calibration samples
     uint64_t startTime = esp_timer_get_time() / 1000;
     int sampleCount = 0;
     
     while ((esp_timer_get_time() / 1000) - startTime < duration * 1000) {
         int micValue = adc1_get_raw(MIC_PIN);
         
         // Add to buffer
         micBuffer[micBufferIndex] = micValue;
         micBufferIndex = (micBufferIndex + 1) % BUFFER_SIZE;
         sampleCount++;
         
         vTaskDelay(50 / portTICK_PERIOD_MS);  // Sample every 50ms
     }
     
     if (sampleCount == 0) {
         ESP_LOGI(TAG, "  No samples collected, using default values");
         return;
     }
     
     // Calculate ambient noise level
     int ambient = getAmbientNoiseLevel();
     int threshold = (ambient * NOISE_MARGIN > MIN_THRESHOLD) ? ambient * NOISE_MARGIN : MIN_THRESHOLD;
     
     ESP_LOGI(TAG, "✓ CALIBRATION COMPLETE");
     ESP_LOGI(TAG, "  • Ambient noise level: %d", ambient);
     ESP_LOGI(TAG, "  • Detection threshold: %d", threshold);
     ESP_LOGI(TAG, "----------------------\n");
     
     // Add a short delay after calibration
     vTaskDelay(1000 / portTICK_PERIOD_MS);
 }