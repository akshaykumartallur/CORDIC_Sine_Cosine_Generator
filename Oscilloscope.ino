#include <Arduino.h>

class CordicCore {
private:
    const uint8_t CORDIC_PI_OVER_2 = 64;
    const uint8_t CORDIC_PI = 128;
    const uint8_t THREE_PI_OVER_2 = 192;
    const int8_t X_INIT = 77;

    int8_t x, y, z;
    int8_t angle_accum;
    uint8_t iter;
    uint8_t Q_reg;
    bool busy;

    const uint8_t atan_table[8] = {32, 19, 10, 5, 3, 1, 1, 0};

public:
    void reset() {
        busy = false;
        x = y = z = 0;
        iter = 0;
        Q_reg = 0;
        angle_accum = 0;
    }

    void compute(uint8_t angle) {
        if (busy) return;

        uint8_t theta_prime;
        if (angle < CORDIC_PI_OVER_2) {
            Q_reg = 0;
            theta_prime = angle;
        } else if (angle < CORDIC_PI) {
            Q_reg = 1;
            theta_prime = angle - CORDIC_PI_OVER_2;
        } else if (angle < THREE_PI_OVER_2) {
            Q_reg = 2;
            theta_prime = angle - CORDIC_PI;
        } else {
            Q_reg = 3;
            theta_prime = angle - THREE_PI_OVER_2;
        }

        busy = true;
        iter = 0;
        x = X_INIT;
        y = 0;
        z = theta_prime;
        angle_accum = 0;
    }

    void update() {
        if (!busy) return;

        if (iter < 8) {
            uint8_t atan_val = atan_table[iter];
            
            if (z >= 0) {
                int8_t x_new = x - (y >> iter);
                int8_t y_new = y + (x >> iter);
                x = x_new;
                y = y_new;
                z -= atan_val;
            } else {
                int8_t x_new = x + (y >> iter);
                int8_t y_new = y - (x >> iter);
                x = x_new;
                y = y_new;
                z += atan_val;
            }
            
            angle_accum += (z >= 0) ? atan_val : -atan_val;
            iter++;
        }

        if (iter == 8) {
            busy = false;
        }
    }

    int8_t getSin() {
        switch (Q_reg) {
            case 0: return y;
            case 1: return x;
            case 2: return -y;
            case 3: return -x;
            default: return 0;
        }
    }

    int8_t getCos() {
        switch (Q_reg) {
            case 0: return x;
            case 1: return -y;
            case 2: return -x;
            case 3: return y;
            default: return 0;
        }
    }

    bool isBusy() const { return busy; }
};
CordicCore cordic;

// ESP32 DAC Pins (Arduino framework uses GPIO numbers)
#define SIN_PIN 25  // DAC1 (GPIO25)
#define COS_PIN 26  // DAC2 (GPIO26)

void setup() {
    Serial.begin(115200);
    
    // No need for explicit DAC enable in Arduino framework
    cordic.reset();
}

void loop() {
    static uint8_t angle = 0;
    
    if (!cordic.isBusy()) {
        cordic.compute(angle);
        angle += 4;  // Adjust this value to change frequency
    }
    
    cordic.update();
    
    if (!cordic.isBusy()) {
        // Convert 8-bit signed (-128 to 127) to 8-bit unsigned (0-255)
        uint8_t sin_value = cordic.getSin() + 128;
        uint8_t cos_value = cordic.getCos() + 128;
        
        // Write directly to DAC pins using Arduino's dacWrite
        dacWrite(SIN_PIN, sin_value);
        dacWrite(COS_PIN, cos_value);
        
        // Optional: print values to serial for debugging
        Serial.printf("Angle: %3u, Sin: %4d, Cos: %4d\n", angle, cordic.getSin(), cordic.getCos());
    }
    
    // Adjust delay to control frequency
    delayMicroseconds(1000);  // Fine-tune this for desired frequency
}
