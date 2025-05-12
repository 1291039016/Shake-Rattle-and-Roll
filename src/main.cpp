#include "mbed.h"
#include <cmath>

// LSM6DSL Register address definitions
#define LSM6DSL_ADDR     (0x6A << 1)  // I2C address (shifted left by 1)
#define LSM6DSL_CTRL1_XL 0x10         // Accelerometer control register
#define LSM6DSL_OUTX_L_XL 0x28        // Accelerometer X-axis low byte output register
#define LSM6DSL_WHO_AM_I  0x0F        // Device ID register

// Frequency Range definitions
#define TREMOR_LOW_FREQ  3    // Tremor minimum frequency
#define TREMOR_HIGH_FREQ 5    // Tremor maximum frequency
#define DYSK_LOW_FREQ    5    // Dyskinesia minimum frequency
#define DYSK_HIGH_FREQ   7    // Dyskinesia maximum frequency

// Tremor intensity levels - uniform steps
#define TREMOR_THRESHOLD_LOW   10    // Tremor detection base threshold
#define TREMOR_THRESHOLD_MED   500   // Tremor medium intensity threshold
#define TREMOR_THRESHOLD_HIGH  1000  // Tremor high intensity threshold

// Dyskinesia intensity levels - uniform steps
#define DYSK_THRESHOLD_LOW   10     // Dyskinesia detection base threshold
#define DYSK_THRESHOLD_MED   700    // Dyskinesia medium intensity threshold
#define DYSK_THRESHOLD_HIGH  1400   // Dyskinesia high intensity threshold

// System state definitions
enum SystemState {
    STATE_IDLE,      // Idle State
    STATE_SAMPLING,  // Data sampling state
    STATE_ANALYZING, // Data analyzing state
    STATE_RESULT     // Result display state
};

// I2C communication
I2C i2c(PB_11, PB_10);  // SDA, SCL pins

// LED indicators
DigitalOut led1(LED1);      // Tremor indicator
DigitalOut led2(LED2);      // Dyskinesia indicator
DigitalOut led_status(LED3); // Status indicator

// Trigger button
DigitalIn trigger_button(BUTTON1); // Assuming BUTTON1 as trigger button

// Timers for LED control
Timeout off1, off2, result_timeout;
void led1_off() { led1 = 0; }
void led2_off() { led2 = 0; }

// System state
SystemState current_state = STATE_IDLE;
bool monitoring_active = false;  // Whether continuous monitoring is active
void return_to_idle() { current_state = STATE_IDLE; }

// Sampling parameters
constexpr int SAMPLE_RATE = 104;  // Sampling rate approx. 104Hz
constexpr int BUF_LEN = SAMPLE_RATE * 3;  // 3 seconds of data
float buf_x[BUF_LEN], buf_y[BUF_LEN], buf_z[BUF_LEN];  // Three-axis data buffers
int idx = 0;  // Current buffer index

// Analysis result variables (global to avoid switch issues)
int trem_x, trem_y, trem_z;
int dysk_x, dysk_y, dysk_z;
int trem_mag, dysk_mag;
int tremor_level, dysk_level;
bool has_tremor, has_dysk;

// Status LED control function
void updateStatusLED() {
    switch(current_state) {
        case STATE_IDLE:
            // Idle State: LED off
            led_status = 0;
            break;
            
        case STATE_SAMPLING:
            // Sampling State: LED on
            led_status = 1;
            break;
            
        case STATE_ANALYZING:
            // Analyzing State: LED on
            led_status = 1;
            break;
            
        case STATE_RESULT:
            // Result Display State: LED off
            led_status = 0;
            break;
    }
}

// I2C register access functions
bool writeRegister(uint8_t reg, uint8_t value) {
    char buf[2] = { (char)reg, (char)value };
    return i2c.write(LSM6DSL_ADDR, buf, 2) == 0;
}

bool readRegisters(uint8_t reg, char* buf, int len) {
    if (i2c.write(LSM6DSL_ADDR, (char*)&reg, 1, true) != 0) return false;
    return i2c.read(LSM6DSL_ADDR, buf, len) == 0;
}

// Calculate band energy - use Goertzel algorithm for efficiency and accuracy
int band_mag(float* data, int k_lo, int k_hi) {
    float total = 0;
    
    // Apply Hanning window to reduce spectral leakage
    float windowed_data[BUF_LEN];
    for (int i = 0; i < BUF_LEN; i++) {
        // Hanning window
        float window = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (BUF_LEN - 1)));
        windowed_data[i] = data[i] * window;
    }
    
    // Goertzel algorithm - more efficient for specific bands
    for (int k = k_lo; k <= k_hi; k++) {
        float omega = 2.0f * M_PI * k / SAMPLE_RATE;
        float cosine = cosf(omega);
        float sine = sinf(omega);
        float coeff = 2.0f * cosine;
        
        float q0 = 0, q1 = 0, q2 = 0;
        
        // Process all samples for this frequency
        for (int n = 0; n < BUF_LEN; n++) {
            q0 = coeff * q1 - q2 + windowed_data[n];
            q2 = q1;
            q1 = q0;
        }
        
        // Compute magnitude
        float real = q1 - q2 * cosine;
        float imag = q2 * sine;
        float magnitude = sqrtf(real*real + imag*imag);
        
        total += magnitude;
    }
    
    // Normalize and scale by number of frequency bins
    return (int)(total / (k_hi - k_lo + 1) * 100.0f);
}

// Get Tremor level (0-3) based on intensity value
int getTremorLevel(int intensity) {
    if (intensity < TREMOR_THRESHOLD_LOW) return 0;      // No symptoms
    if (intensity < TREMOR_THRESHOLD_MED) return 1;      // Mild
    if (intensity < TREMOR_THRESHOLD_HIGH) return 2;     // Moderate
    return 3;                                            // Severe
}

// Get Dyskinesia level (0-3) based on intensity value
int getDyskinesiaLevel(int intensity) {
    if (intensity < DYSK_THRESHOLD_LOW) return 0;       // No symptoms
    if (intensity < DYSK_THRESHOLD_MED) return 1;       // Mild
    if (intensity < DYSK_THRESHOLD_HIGH) return 2;      // Moderate
    return 3;                                           // Severe
}

// Analyze data and display result
void analyzeData()
{
    printf("--- Analyzing 3-second data window ---\n");

    // Calculate mean and remove DC component
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < BUF_LEN; i++) {
        sum_x += buf_x[i];
        sum_y += buf_y[i];
        sum_z += buf_z[i];
    }

    float mean_x = sum_x / BUF_LEN;
    float mean_y = sum_y / BUF_LEN;
    float mean_z = sum_z / BUF_LEN;

    for (int i = 0; i < BUF_LEN; i++) {
        buf_x[i] -= mean_x;
        buf_y[i] -= mean_y;
        buf_z[i] -= mean_z;
    }

    int max_amp_x = 0, max_amp_y = 0, max_amp_z = 0;
    for (int i = 0; i < BUF_LEN; i++) {
        int amp_x = (int)(fabsf(buf_x[i]) * 1000);
        int amp_y = (int)(fabsf(buf_y[i]) * 1000);
        int amp_z = (int)(fabsf(buf_z[i]) * 1000);

        if (amp_x > max_amp_x) max_amp_x = amp_x;
        if (amp_y > max_amp_y) max_amp_y = amp_y;
        if (amp_z > max_amp_z) max_amp_z = amp_z;
    }

    printf("Max amplitude: X=%d, Y=%d, Z=%d mg\n", max_amp_x, max_amp_y, max_amp_z);

    trem_x = band_mag(buf_x, TREMOR_LOW_FREQ, TREMOR_HIGH_FREQ);
    trem_y = band_mag(buf_y, TREMOR_LOW_FREQ, TREMOR_HIGH_FREQ);
    trem_z = band_mag(buf_z, TREMOR_LOW_FREQ, TREMOR_HIGH_FREQ);

    dysk_x = band_mag(buf_x, DYSK_LOW_FREQ, DYSK_HIGH_FREQ);
    dysk_y = band_mag(buf_y, DYSK_LOW_FREQ, DYSK_HIGH_FREQ);
    dysk_z = band_mag(buf_z, DYSK_LOW_FREQ, DYSK_HIGH_FREQ);

    trem_mag = std::max({trem_x, trem_y, trem_z});
    dysk_mag = std::max({dysk_x, dysk_y, dysk_z});

    tremor_level = getTremorLevel(trem_mag);
    dysk_level = getDyskinesiaLevel(dysk_mag);

    printf("Tremor levels: None(<%d), Mild(%d-%d), Moderate(%d-%d), Severe(>%d)\n",
           TREMOR_THRESHOLD_LOW,
           TREMOR_THRESHOLD_LOW, TREMOR_THRESHOLD_MED - 1,
           TREMOR_THRESHOLD_MED, TREMOR_THRESHOLD_HIGH - 1,
           TREMOR_THRESHOLD_HIGH);

    printf("Dyskinesia levels: None(<%d), Mild(%d-%d), Moderate(%d-%d), Severe(>%d)\n",
           DYSK_THRESHOLD_LOW,
           DYSK_THRESHOLD_LOW, DYSK_THRESHOLD_MED - 1,
           DYSK_THRESHOLD_MED, DYSK_THRESHOLD_HIGH - 1,
           DYSK_THRESHOLD_HIGH);

    printf("Tremor intensity = %d (X=%d, Y=%d, Z=%d), level = %d\n",
           trem_mag, trem_x, trem_y, trem_z, tremor_level);

    printf("Dyskinesia intensity = %d (X=%d, Y=%d, Z=%d), level = %d\n",
           dysk_mag, dysk_x, dysk_y, dysk_z, dysk_level);

    has_tremor = (tremor_level > 0);
    has_dysk = (dysk_level > 0);

    // LED Blinking Parameters
    const int BLINK_DURATION = 150; // Duration of each blink in ms
    const int BLINK_COUNT = 2;      // Number of blinks per level
    const int CYCLE_DURATION = 2 * BLINK_DURATION; // One full blink cycle (on + off)

    if (has_tremor && (!has_dysk || trem_mag > dysk_mag)) {
        const char* level_desc[] = {"None", "Mild", "Moderate", "Severe"};
        printf("→ Tremor detected, intensity=%d (%s), Frequency: %d-%dHz\n",
               trem_mag, level_desc[tremor_level], TREMOR_LOW_FREQ, TREMOR_HIGH_FREQ);

        // Calculate total blinking time
        int total_blinks = tremor_level * BLINK_COUNT;
        float total_duration_s = (total_blinks * CYCLE_DURATION) / 1000.0f;

        // Alternating LED pattern for Tremor
        for (int i = 0; i < total_blinks; i++) {
            led1 = 1; led2 = 0; // LED1 on, LED2 off
            ThisThread::sleep_for(chrono::milliseconds(BLINK_DURATION));
            led1 = 0; led2 = 1; // LED1 off, LED2 on
            ThisThread::sleep_for(chrono::milliseconds(BLINK_DURATION));
        }
        led1 = 0; led2 = 0; // Turn off both LEDs

        // Set timeout to ensure full sequence completes, with a small buffer
        off1.attach(callback(led1_off), total_duration_s + 0.5f);
        off2.attach(callback(led2_off), total_duration_s + 0.5f);

    } else if (has_dysk) {
        const char* level_desc[] = {"None", "Mild", "Moderate", "Severe"};
        printf("→ Dyskinesia detected, intensity=%d (%s), Frequency: %d-%dHz\n",
               dysk_mag, level_desc[dysk_level], DYSK_LOW_FREQ, DYSK_HIGH_FREQ);

        // Calculate total blinking time
        int total_blinks = dysk_level * BLINK_COUNT;
        float total_duration_s = (total_blinks * CYCLE_DURATION) / 1000.0f;

        // Simultaneous LED pattern for Dyskinesia
        for (int i = 0; i < total_blinks; i++) {
            led1 = 1; led2 = 1; // Both LEDs on
            ThisThread::sleep_for(chrono::milliseconds(BLINK_DURATION));
            led1 = 0; led2 = 0; // Both LEDs off
            ThisThread::sleep_for(chrono::milliseconds(BLINK_DURATION));
        }
        led1 = 0; led2 = 0; // Turn off both LEDs

        // Set timeout to ensure full sequence completes, with a small buffer
        off1.attach(callback(led1_off), total_duration_s + 0.5f);
        off2.attach(callback(led2_off), total_duration_s + 0.5f);

    } else {
        printf("→ No significant symptoms detected\n");
        led1 = 0;
        led2 = 0;
    }

    printf("--- Analysis complete ---\n\n");
}

int main() {
    printf("Starting Parkinson's symptom detection system\n");
    
    // Initialize I2C
    i2c.frequency(400000);  // Set I2C frequency to 400kHz
    
    // Configure accelerometer - set to 104Hz, ±2g
    if (!writeRegister(LSM6DSL_CTRL1_XL, 0x60)) {
        printf("Error: Accelerometer configuration failed\n");
        return 1;
    }
    
    // Read Device ID to confirm communication
    char who = 0;
    readRegisters(LSM6DSL_WHO_AM_I, &who, 1);
    printf("Device ID = 0x%02X (expected 0x6A)\n", who);
    
    // Initialize LEDs
    led1 = 0;
    led2 = 0;
    led_status = 0;
    
    // Configure button
    trigger_button.mode(PullUp); // Set to pull-up mode, pressed = low
    
    printf("System ready. Press the button to start/stop monitoring\n");
    
    bool button_released = true; // Used to detect button release
    
    // Main loop
    while (true) {
        // Update status LED
        updateStatusLED();
        
        // Handle button input - toggle mode
        if (trigger_button.read() == 0) {
            // Button is pressed
            if (button_released) {
                // Toggle monitoring status
                monitoring_active = !monitoring_active;
                
                if (monitoring_active) {
                    printf("Starting continuous monitoring mode\n");
                    // Clear buffer and initialize
                    idx = 0;
                    // Switch to Sampling State
                    current_state = STATE_SAMPLING;
                } else {
                    printf("Stopping monitoring mode\n");
                    // Switch to Idle State
                    current_state = STATE_IDLE;
                    // Turn off all LEDs
                    led1 = 0;
                    led2 = 0;
                    led_status = 0;
                }
                
                // Prevent button debounce or retrigger
                ThisThread::sleep_for(200ms);
                
                // Mark button as pressed
                button_released = false;
            }
        } else {
            // Button released
            button_released = true;
        }
        
        // Execute different actions based on current state
        switch (current_state) {
            case STATE_IDLE:
                // Idle State, no data collection
                break;
                
            case STATE_SAMPLING: {
                // Sampling State, collect sensor data
                char raw[6];
                if (readRegisters(LSM6DSL_OUTX_L_XL, raw, 6)) {
                    // Convert to 3-axis acceleration data
                    int16_t ax_raw = (raw[1] << 8) | raw[0];
                    int16_t ay_raw = (raw[3] << 8) | raw[2];
                    int16_t az_raw = (raw[5] << 8) | raw[4];
                    
                    // Convert to g-units and store
                    buf_x[idx] = ax_raw * (2.0f / 32768.0f);
                    buf_y[idx] = ay_raw * (2.0f / 32768.0f);
                    buf_z[idx] = az_raw * (2.0f / 32768.0f);
                    
                    idx++;
                    
                    // When 3 seconds of data is collected, switch to Analyzing State
                    if (idx >= BUF_LEN) {
                        printf("\nData collection complete, starting analysis...\n");
                        current_state = STATE_ANALYZING;
                    }
                } else {
                    printf("Failed to read sensor data\n");
                }
                break;
            }
                
            case STATE_ANALYZING:
                // Analyzing State, process collected data
                analyzeData();
                
                // If still in monitoring mode, return to Sampling State for next collection
                if (monitoring_active) {
                    // Reset index for next sampling
                    idx = 0;
                    // Return to Sampling State
                    current_state = STATE_SAMPLING;
                } else {
                    // Switch to Result Display State
                    current_state = STATE_RESULT;
                    // Set timeout to return to Idle State after 5 seconds
                    result_timeout.attach(callback(return_to_idle), 5s);
                }
                break;
                
            case STATE_RESULT:
                // Result Display State, wait for timeout to return to Idle State
                break;
        }
        
        // Wait for next loop
        ThisThread::sleep_for(10ms);
    }
}