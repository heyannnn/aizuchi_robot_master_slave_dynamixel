// ============================================================================
// LIBRARIES
// ============================================================================
#include <M5Unified.h>           // M5Stack hardware library
#include <Dynamixel2Arduino.h>   // ROBOTIS library for Dynamixel motor control
#include <UNIT_8ENCODER.h>       // M5Stack 8-Encoder Unit library


// The flow: M5Stack (TTL) → DMX Base (TTL→RS485) → Motor (RS485)
// M5Stack DMX Base Pin Configuration for CoreS3:
#define DXL_SERIAL Serial1       // Use Serial1 hardware UART
#define TXD_PIN 7                // GPIO 7 = TX (transmit data to DMX Base)
#define RXD_PIN 10               // GPIO 10 = RX (receive data from DMX Base)
#define DXL_DIR_PIN 6            // GPIO 6 = EN (direction control for RS485)
                                 // EN pin controls TX/RX mode on RS485 chip

// ============================================================================
// DYNAMIXEL MOTOR SETTINGS
// ============================================================================
//const uint8_t DXL_ID = 1;                    // Motor ID (set in motor, default 1)
// Master motors (people control by hand)
const uint8_t MASTER_IDS[] = {10, 11, 12};
// Slave motors (controlled by M5 to follow masters)
const uint8_t SLAVE_IDS[] = {20, 21, 22};
const int NUM_MOTORS = 3;
const float DXL_PROTOCOL_VERSION = 2.0;      // Protocol 2.0 for XL430
const uint32_t DXL_BAUD_RATE = 1000000;      // Communication speed (bits/second)
//const uint32_t DXL_BAUD_RATE = 57600;
                                             // RS485 supports: 9600, 57600, 115200, 1000000, etc.
                                             // 1000000 = 1 Mbps (modern Dynamixel default) Dynamixel XL430 W250-T
                                             // 57600 = 1 Mbps (modern Dynamixel default) Dynamixel XL430 W250-T
                                             // Your motor is configured to 1000000 baud

// Create Dynamixel controller object
// This handles all the complex packet formatting and RS485 communication
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Control Table Addresses for XW540 (Protocol 2.0)
#define ADDR_TORQUE_ENABLE 64
#define ADDR_HARDWARE_ERROR_STATUS 70
#define ADDR_MOVING 122

// ============================================================================
// ENCODER SETUP - M5Stack 8-Encoder Unit
// ============================================================================
#define ENCODER_I2C_ADDR 0x41    // I2C address for 8-Encoder Unit
UNIT_8ENCODER encoder;           // Create encoder object

// ============================================================================
// MOTION CONTROL VARIABLES
// ============================================================================
bool encoder_found = false;        // Flag: encoder detected and working?

int polling_rate = 60;              // Default 60Hz
const int MIN_POLLING_RATE = 10;    // Minimum 10Hz
const int MAX_POLLING_RATE = 120;   // Maximum 120Hz
unsigned long last_poll_time = 0;



// ============================================================================
// SETUP - Runs once when M5Stack starts
// ============================================================================
void setup() {
    // Initialize M5Stack hardware AFTER I2C cleanup
    auto cfg = M5.config();
    M5.begin(cfg);
    delay(200);                   // wait a bit
    M5.Power.setExtOutput(true);  // turn on Grove 5V, 

    for (int i = 0; i < 3; i++) {
        Wire.begin(2, 1);
        encoder.begin(&Wire, 0x41, 2, 1);
        Wire.beginTransmission(0x00);
        Wire.write(0x06);  // General call reset
        Wire.endTransmission();
        Wire.beginTransmission(ENCODER_I2C_ADDR);
        Wire.endTransmission();
        Wire.end();
    }

    // Final I2C initialization
    Wire.begin(2, 1);
    // delay(100);

    // Setup display
    M5.Display.setTextSize(1.5);
    M5.Display.setCursor(10, 10);
    M5.Display.println("Motor + Encoder Control");
    M5.Display.setCursor(10, 30);
    M5.Display.println("Starting...");
    delay(100);

    // Try to initialize encoder with retries
    encoder_found = false;
    for (int retry = 0; retry < 25; retry++) {  // Reduced from 50 to 25
        // Every 5th retry, do a full I2C reset
        if (retry > 0 && retry % 5 == 0) {
            M5.Display.setCursor(10, 45);
            M5.Display.printf("Connecting encorder...", retry);
            M5.Display.setCursor(10, 60);
            M5.Display.println("Please replug encoder");
            // Complete I2C bus reset
            Wire.end();
            delay(300);  // Reduced from 1000ms

            // Reinitialize I2C
            Wire.begin(2, 1);
            delay(300);  // Reduced from 1000ms

            // Try sending general call reset to I2C bus (address 0x00)
            Wire.beginTransmission(0x00);
            Wire.write(0x06);  // General call reset command
            Wire.endTransmission();
            delay(200);  // Reduced from 500ms
        }

        // Try to initialize encoder
        if (encoder.begin(&Wire, ENCODER_I2C_ADDR, 2, 1)) {
            for (int reset_attempt = 0; reset_attempt < 5; reset_attempt++) {
                for (int ch = 0; ch < 8; ch++) {
                    encoder.resetCounter(ch);
                }
                delay(50);
            }

            // Set CH1 to wall position instead of 0
            encoder.setEncoderValue(0, 0);

            delay(100);
            int32_t ch1_check = encoder.getEncoderValue(0);

            encoder_found = true;
            break;  // Success - exit retry loop
        }

        delay(200);  // Reduced from 400ms
    }

    if (!encoder_found) {
        M5.Display.clear();
        M5.Display.setCursor(10, 10);
        M5.Display.println("WARNING: No Encoder");
        M5.Display.setCursor(10, 30);
        M5.Display.println("Using default 60Hz");
        delay(2000);
    }

    // Initialize Serial for debug
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Starting System ===");

    Serial1.begin(DXL_BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);

    dxl.begin(DXL_BAUD_RATE);

    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // ========================================================================
    // FIND AND CONFIGURE MOTOR
    // ========================================================================
    M5.Display.setCursor(10, 50);
    M5.Display.println("Initializing motors...");
    Serial.println("=== Motor Init ===");

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Check master motor
        Serial.printf("Pinging Master %d...", MASTER_IDS[i]);
        if (dxl.ping(MASTER_IDS[i])) {
            Serial.println("FOUND");
            M5.Display.setCursor(10, 70 + i*20);
            M5.Display.setTextColor(GREEN);
            M5.Display.printf("Master %d: OK", MASTER_IDS[i]);

            dxl.torqueOff(MASTER_IDS[i]);
            delay(50);
            dxl.setOperatingMode(MASTER_IDS[i], OP_POSITION);
            delay(50);
            dxl.torqueOn(MASTER_IDS[i]);
        } else {
            Serial.println("FAILED");
            M5.Display.setCursor(10, 70 + i*20);
            M5.Display.setTextColor(RED);
            M5.Display.printf("Master %d: FAIL", MASTER_IDS[i]);
        }

        // Check slave motor
        Serial.printf("Pinging Slave %d...", SLAVE_IDS[i]);
        if (dxl.ping(SLAVE_IDS[i])) {
            Serial.println("FOUND");
            M5.Display.setCursor(10, 150 + i*20);
            M5.Display.setTextColor(GREEN);
            M5.Display.printf("Slave %d: OK", SLAVE_IDS[i]);

            dxl.torqueOff(SLAVE_IDS[i]);
            delay(50);
            dxl.setOperatingMode(SLAVE_IDS[i], OP_POSITION);
            delay(50);
            dxl.torqueOn(SLAVE_IDS[i]);
        } else {
            Serial.println("FAILED");
            M5.Display.setCursor(10, 150 + i*20);
            M5.Display.setTextColor(RED);
            M5.Display.printf("Slave %d: FAIL", SLAVE_IDS[i]);
        }
    }

    M5.Display.setTextColor(WHITE);

    delay(2000);  // Show motor init status for 2 seconds
    M5.Display.clear();  // Clear once before entering loop
}


    // ========================================================================
    // HOMING: Move motor to wall position if outside safe range
    // ========================================================================

    void displayInfo() {
        static unsigned long last_display_update = 0;
        if (millis() - last_display_update < 100) return;  // Update every 100ms (faster)
        last_display_update = millis();

        M5.Display.setCursor(10, 10);
        M5.Display.printf("=== Master-Slave Control ===");

        // Clear and display polling rate
        M5.Display.fillRect(10, 40, 300, 20, BLACK);
        M5.Display.setCursor(10, 40);
        M5.Display.printf("Polling Rate: %d Hz", polling_rate);

        if (encoder_found) {
            int32_t encoder_ch1 = encoder.getEncoderValue(0);
            // Clear and display encoder value
            M5.Display.fillRect(10, 60, 300, 20, BLACK);
            M5.Display.setCursor(10, 60);
            M5.Display.printf("Encoder CH1: %ld", encoder_ch1);
        }

        // Display master and slave positions
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t master_pos = dxl.getPresentPosition(MASTER_IDS[i]);
            int32_t slave_pos = dxl.getPresentPosition(SLAVE_IDS[i]);

            // Clear master line
            M5.Display.fillRect(10, 90 + i*40, 300, 20, BLACK);
            M5.Display.setCursor(10, 90 + i*40);
            M5.Display.setTextColor(WHITE);
            M5.Display.printf("M%d: %ld", MASTER_IDS[i], master_pos);

            // Clear slave line
            M5.Display.fillRect(10, 110 + i*40, 300, 20, BLACK);
            M5.Display.setCursor(10, 110 + i*40);
            M5.Display.setTextColor(GREEN);
            M5.Display.printf("S%d: %ld", SLAVE_IDS[i], slave_pos);
        }
        M5.Display.setTextColor(WHITE);
    }

// ============================================================================
// LOOP - Runs continuously (many times per second)
// ============================================================================
  void loop() {
      M5.update();

      // Calculate polling delay based on polling rate
      unsigned long poll_interval = 1000 / polling_rate;  // Convert Hz to milliseconds

      // Only poll at the specified rate
      if (millis() - last_poll_time < poll_interval) {
          return;  // Not time to poll yet
      }
      last_poll_time = millis();

      // Read encoder CH1 for polling rate control (if encoder found)
      if (encoder_found) {
          int32_t encoder_ch1_value = encoder.getEncoderValue(0);

          // Map encoder value to polling rate (±20 range → 10-120Hz)
          polling_rate = 60 + (encoder_ch1_value * 2);  // Start at 60Hz  default
          if (polling_rate < MIN_POLLING_RATE) polling_rate =  MIN_POLLING_RATE;
          if (polling_rate > MAX_POLLING_RATE) polling_rate =  MAX_POLLING_RATE;
      }

      // Read master positions and write to slaves
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t master_pos = dxl.getPresentPosition(MASTER_IDS[i]);
            int32_t master_vel = dxl.getPresentVelocity(MASTER_IDS[i]);

            dxl.setGoalPosition(SLAVE_IDS[i], master_pos);
            dxl.setProfileVelocity(SLAVE_IDS[i], abs(master_vel) + 50); // Match speed + margin
        }

      // Display info
      displayInfo();
  }

  