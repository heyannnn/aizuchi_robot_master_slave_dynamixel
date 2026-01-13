// ============================================================================
// SIMPLE MOTOR TEST - Test single motor ID 10
// ============================================================================
#include <M5Unified.h>
#include <Dynamixel2Arduino.h>

// Pin configuration
#define DXL_SERIAL Serial1
#define TXD_PIN 7
#define RXD_PIN 10
#define DXL_DIR_PIN 6

// Motor settings
const uint8_t TEST_MOTOR_ID = 11;
const float DXL_PROTOCOL_VERSION = 2.0;
//const uint32_t DXL_BAUD_RATE = 57600;
const uint32_t DXL_BAUD_RATE = 1000000;

// Create Dynamixel controller
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Control table addresses
#define ADDR_TORQUE_ENABLE 64

void setup() {
    // Initialize M5Stack
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Power.setExtOutput(true);

    // Try different baudrates
    uint32_t baudrates[] = {57600, 115200, 1000000, 2000000};
    for (int i = 0; i < 4; i++) {
        Serial.printf("Trying baudrate: %d\n", baudrates[i]);
        M5.Display.setCursor(10, 50 + i*20);
        M5.Display.printf("Trying: %d", baudrates[i]);

        Serial1.end();
        delay(100);
        Serial1.begin(baudrates[i], SERIAL_8N1, RXD_PIN, TXD_PIN);
        dxl.begin(baudrates[i]);

        if (dxl.ping(TEST_MOTOR_ID)) {
            Serial.printf("FOUND at baudrate: %d\n", baudrates[i]);
            M5.Display.printf(" - FOUND!");
            break;
        }
    }

    // Setup display
    M5.Display.clear();
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.println("Motor Test ID 10");

    // Initialize serial for debug
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Motor Test Starting ===");

    // Initialize Dynamixel
    Serial1.begin(DXL_BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);
    dxl.begin(DXL_BAUD_RATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
 
    M5.Display.setCursor(10, 40);
    M5.Display.println("Pinging motor...");
    Serial.println("Pinging motor ID 10...");

    // Try to ping motor
    if (dxl.ping(TEST_MOTOR_ID)) {
        M5.Display.setCursor(10, 70);
        M5.Display.setTextColor(GREEN);
        M5.Display.println("Motor 10 FOUND!");
        Serial.println("SUCCESS: Motor 10 found!");

        // Get model number
        uint16_t model = dxl.getModelNumber(TEST_MOTOR_ID);
        M5.Display.setCursor(10, 100);
        M5.Display.printf("Model: %d", model);
        Serial.printf("Model number: %d\n", model);

        // Configure motor
        M5.Display.setCursor(10, 130);
        M5.Display.println("Configuring...");
        Serial.println("Configuring motor...");

        // Try torque off
        bool result1 = dxl.torqueOff(TEST_MOTOR_ID);
        Serial.printf("Torque OFF result: %d\n", result1);
        delay(300);

        // Try set operating mode
        bool result2 = dxl.setOperatingMode(TEST_MOTOR_ID, OP_POSITION);
        Serial.printf("Set mode result: %d\n", result2);
        delay(300);

        // Read current position
        int32_t current_pos = dxl.getPresentPosition(TEST_MOTOR_ID);
        M5.Display.setCursor(10, 160);
        M5.Display.printf("Pos: %ld", current_pos);
        Serial.printf("Current position: %ld\n", current_pos);

        // Turn on torque
        dxl.torqueOn(TEST_MOTOR_ID);
        delay(100);

        // Check if torque is on
        uint8_t torque_status = dxl.readControlTableItem(ADDR_TORQUE_ENABLE, TEST_MOTOR_ID);
        M5.Display.setCursor(10, 190);
        if (torque_status == 1) {
            M5.Display.setTextColor(GREEN);
            M5.Display.println("Torque ON!");
            Serial.println("Torque enabled successfully");
        } else {
            M5.Display.setTextColor(RED);
            M5.Display.println("Torque FAILED!");
            Serial.println("ERROR: Torque not enabled");
        }

        M5.Display.setCursor(10, 220);
        M5.Display.setTextColor(WHITE);
        M5.Display.println("Ready to test!");

    } else {
        M5.Display.setCursor(10, 70);
        M5.Display.setTextColor(RED);
        M5.Display.println("Motor NOT FOUND!");
        Serial.println("ERROR: Motor 10 not responding");

        M5.Display.setCursor(10, 100);
        M5.Display.setTextColor(WHITE);
        M5.Display.println("Check:");
        M5.Display.setCursor(10, 120);
        M5.Display.println("- Wiring");
        M5.Display.setCursor(10, 140);
        M5.Display.println("- Power 12V");
        M5.Display.setCursor(10, 160);
        M5.Display.println("- Motor ID=10");
        M5.Display.setCursor(10, 180);
        M5.Display.println("- Baudrate=1M");
    }

    delay(3000);
}

void loop() {
    M5.update();

    // Only run if motor was found
    static bool motor_found = dxl.ping(TEST_MOTOR_ID);
    if (!motor_found) {
        delay(1000);
        return;
    }

    // Read current position
    int32_t current_pos = dxl.getPresentPosition(TEST_MOTOR_ID);

    // Move back and forth between two positions
    static unsigned long last_move = 0;
    static bool direction = true;

    if (millis() - last_move > 2000) {  // Move every 2 seconds
        last_move = millis();

        int32_t target_pos;
        if (direction) {
            target_pos = current_pos + 512;  // Move +45 degrees
        } else {
            target_pos = current_pos - 512;  // Move -45 degrees
        }

        direction = !direction;

        M5.Display.clear();
        M5.Display.setCursor(10, 10);
        M5.Display.println("=== Motor Test ===");
        M5.Display.setCursor(10, 50);
        M5.Display.printf("Current: %ld", current_pos);
        M5.Display.setCursor(10, 80);
        M5.Display.printf("Target:  %ld", target_pos);

        Serial.printf("Moving from %ld to %ld\n", current_pos, target_pos);

        dxl.setGoalPosition(TEST_MOTOR_ID, target_pos);

        M5.Display.setCursor(10, 110);
        M5.Display.println("Moving...");
    }

    delay(100);
}
