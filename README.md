Project Summary: For our Real Time Embedded Systems (ECE-GY - 6483) Team Challenge, we designed a portable device to detect and visually differentiate between Parkinson’s disease symptoms—tremors (3-5Hz) and dyskinesia (5-7Hz)—using accelerometer/gyroscope data.

Detection Method:
- Captured 3-second sensor data intervals and analyzed them with an FFT library to isolate tremor (3-5Hz) and dyskinesia (5-7Hz) frequency bands.
- Quantified intensity based on the amplitude of dominant frequencies.

Creative Visual Feedback:
- Tremors: Triggered an alternating LED blinking pattern (e.g., left-right-left) where the blinking times increased with tremor intensity (2, 4, 6 times for mild, moderate, and severe tremors, respectively).
- Dyskinesia: Activated a synchronous LED blinking pattern (all LEDs on/off together) where the blinking times increased with dyskinesia severity (2, 4, 6 times for mild, moderate, and severe dyskinesia, respectively).

Implementation:
- Built entirely on a microcontroller development board STM32L4 (no external hardware).
- Powered by a USB battery pack for portability.
- Coded in C/C++ using PlatformIO, ensuring efficient real-time signal processing.

Impact: This system provides immediate, intuitive feedback to help patients and caregivers distinguish between under-medicated ("Off") and over-medicated ("Over-On") states, enabling better dopamine dosage adjustments.

Key Innovation: Transforming raw sensor data into dynamic visual patterns that reflect both the type (alternating vs. synchronous) and intensity of symptoms, all while adhering to strict hardware/software constraints.

You can have a look at a demo [here](https://drive.google.com/file/d/1T8wZCR-3Rk6CA5qGjVE1vUXUwb3xCGly/view?usp=sharing).
