/*
 * ESP32 LoRa Controller Firmware for Drone
 * ----------------------------------------
 * Reads controller inputs (buttons/sticks) and transmits them over LoRa
 * to the SBC's LoRaControllerBackend using the CMD_SET_POSITION frame format.
 *
 * Supported boards: Heltec WiFi LoRa 32 V3/V4, LilyGO T3-S3, generic ESP32+SX1262
 *
 * Pin Configuration (Heltec V3/V4 default):
 *   SX1262 SPI: SCLK=9, MOSI=10, MISO=11, CS=8, RST=12, BUSY=13, DIO1=14
 *   Controller: custom GPIOs for buttons/sticks (see config below)
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ==================== CONFIGURATION ====================
// LoRa Settings
#define LORA_FREQUENCY   915.0
#define LORA_TX_POWER    20
#define LORA_SPREADING_FACTOR  10
#define LORA_BANDWIDTH   125.0
#define LORA_CODING_RATE 5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYNC_WORD   0x12
#define LORA_CRC         true

// Controller Pin Mapping (adjust for your board)
#define BTN_M1_PIN      15
#define BTN_M2_PIN      16
#define BTN_N1_PIN      17
#define BTN_N2_PIN      18
#define BTN_A_PIN       19
#define BTN_B_PIN       21
#define BTN_X_PIN       22
#define BTN_Y_PIN       23
#define BTN_LB_PIN      25
#define BTN_RB_PIN      26
#define BTN_BACK_PIN    27
#define BTN_START_PIN   32
#define BTN_DPAD_UP     33
#define BTN_DPAD_DOWN   34
#define BTN_DPAD_LEFT   35
#define BTN_DPAD_RIGHT  36

// Joystick ADC pins (analog inputs)
#define JOY_LEFT_X_PIN   34   // Left stick horizontal
#define JOY_LEFT_Y_PIN   35   // Left stick vertical
#define JOY_RIGHT_X_PIN  32   // Right stick horizontal
#define JOY_RIGHT_Y_PIN  33   // Right stick vertical

// Communication settings
#define LORA_MSG_TYPE    0x06  // CMD_SET_POSITION (reused for controller state)
#define TX_INTERVAL_MS   100   // 10 Hz controller update rate
#define HEARTBEAT_INTERVAL_MS 5000
#define DEVICE_ID        12345 // Match SBC config

// Protocol constants (must match SBC lora_protocol.py)
#define FRAME_HEADER_LEN 10
#define FRAME_PAYLOAD_LEN 26  // 4 floats + 2 uint32 + uint16 = 26 bytes
#define FRAME_DEV_ID_LEN 4
#define FRAME_HMAC_LEN 32     // For security
#define MAX_FRAME_LEN 200

// ==================== GLOBAL STATE ====================
SPIClass spiLoRa;
SX1262 radio = new Module(8, 14, 12, 13, spiLoRa, SPISettings(8000000, MSBFIRST, SPI_MODE0));

// Controller state
struct ControllerState {
    int16_t roll;     // -32768 to 32767
    int16_t pitch;    // -32768 to 32767
    int16_t yaw;      // -32768 to 32767
    int16_t throttle; // -32768 to 32767
    uint32_t buttons_low;   // Standard buttons 0-15
    uint32_t buttons_high;  // Standard buttons 16-31
    uint16_t buttons_extra; // M1, M2, N1, N2
    uint32_t timestamp;
};

// Last sent values to avoid redundant transmissions
struct LastSentState {
    int16_t roll, pitch, yaw, throttle;
    uint32_t buttons_low, buttons_high;
    uint16_t buttons_extra;
} lastSent;

// Button state tracking (debouncing)
uint8_t buttonState[17]; // Last read state for each button
uint8_t lastButtonState[17];
uint32_t buttonDebounceTime[17];

// ==================== PROTOTYPES ====================
bool initLoRa();
bool readController();
void sendControllerState();
uint16_t readADC(uint8_t pin);
void updateButtons();
void handleButton(int btn, int pin);
float adcToStick(uint16_t adc);
uint16_t stickToRaw(float value);

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== ESP32 LoRa Controller ===");

    // Configure controller pins
    pinMode(BTN_M1_PIN, INPUT_PULLUP);
    pinMode(BTN_M2_PIN, INPUT_PULLUP);
    pinMode(BTN_N1_PIN, INPUT_PULLUP);
    pinMode(BTN_N2_PIN, INPUT_PULLUP);
    pinMode(BTN_A_PIN, INPUT_PULLUP);
    pinMode(BTN_B_PIN, INPUT_PULLUP);
    pinMode(BTN_X_PIN, INPUT_PULLUP);
    pinMode(BTN_Y_PIN, INPUT_PULLUP);
    pinMode(BTN_LB_PIN, INPUT_PULLUP);
    pinMode(BTN_RB_PIN, INPUT_PULLUP);
    pinMode(BTN_BACK_PIN, INPUT_PULLUP);
    pinMode(BTN_START_PIN, INPUT_PULLUP);
    pinMode(BTN_DPAD_UP, INPUT_PULLUP);
    pinMode(BTN_DPAD_DOWN, INPUT_PULLUP);
    pinMode(BTN_DPAD_LEFT, INPUT_PULLUP);
    pinMode(BTN_DPAD_RIGHT, INPUT_PULLUP);

    // Initialize LoRa
    if (!initLoRa()) {
        Serial.println("LoRa init failed! Check wiring.");
        while (1) delay(1000);  // Halt
    }

    Serial.println("Controller ready! Sending at " + String(TX_INTERVAL_MS) + "ms intervals.");
}

// ==================== MAIN LOOP ====================
void loop() {
    static uint32_t lastTx = 0;
    static uint32_t lastHb = 0;

    uint32_t now = millis();

    // Read controller at high rate
    if (readController()) {
        // Send if changed significantly or heartbeat time
        if (now - lastTx >= TX_INTERVAL_MS) {
            sendControllerState();
            lastTx = now;
        }
    }

    // Send heartbeat periodically (empty controller state = safety)
    if (now - lastHb >= HEARTBEAT_INTERVAL_MS) {
        // Heartbeat could be a special frame type or just repeated state
        sendControllerState();
        lastHb = now;
    }
}

// ==================== LORA INIT ====================
bool initLoRa() {
    Serial.print("Initializing SX1262...");

    // Configure SPI pins
    spiLoRa.begin(9, 11, 10, 8);  // SCLK, MISO, MOSI, SS

    // Initialize radio
    int state = radio.begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE_LENGTH,
        0  // CRC
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("failed (");
        Serial.print(state);
        Serial.println(")");
        return false;
    }

    Serial.println(" OK");
    return true;
}

// ==================== CONTROLLER READ ====================
bool readController() {
    // Read joystick ADCs
    uint16_t leftX = readADC(JOY_LEFT_X_PIN);
    uint16_t leftY = readADC(JOY_LEFT_Y_PIN);
    uint16_t rightX = readADC(JOY_RIGHT_X_PIN);
    uint16_t rightY = readADC(JOY_RIGHT_Y_PIN);

    // Convert to -32768 to 32767 range (centered at 0)
    lastSent.roll = adcToStick(leftX);
    lastSent.pitch = adcToStick(leftY);
    lastSent.yaw = adcToStick(rightX);
    lastSent.throttle = adcToStick(rightY);

    // Read buttons with debouncing
    updateButtons();

    return true;
}

uint16_t readADC(uint8_t pin) {
    // For analog joystick reading
    // ESP32 ADC range: 0-4095 (12-bit) or 0-4095 on ADC1
    return analogRead(pin);
}

float adcToStick(uint16_t adc) {
    // Convert ADC value (0-4095) to float (-1.0 to 1.0)
    return (adc - 2047.5) / 2047.5;
}

uint16_t stickToRaw(float value) {
    // Convert -1.0..1.0 to -32768..32767 (int16_t)
    return (int16_t)(value * 32767);
}

// ==================== BUTTON HANDLING ====================
void updateButtons() {
    uint32_t now = millis();
    const uint8_t DEBOUNCE_MS = 20;

    // Handle all buttons
    handleButton(0, BTN_A_PIN);
    handleButton(1, BTN_B_PIN);
    handleButton(2, BTN_X_PIN);
    handleButton(3, BTN_Y_PIN);
    handleButton(4, BTN_LB_PIN);
    handleButton(5, BTN_RB_PIN);
    handleButton(6, BTN_BACK_PIN);
    handleButton(7, BTN_START_PIN);
    handleButton(8, BTN_DPAD_UP);
    handleButton(9, BTN_DPAD_DOWN);
    handleButton(10, BTN_DPAD_LEFT);
    handleButton(11, BTN_DPAD_RIGHT);
    handleButton(14, BTN_M1_PIN);
    handleButton(15, BTN_M2_PIN);
    handleButton(16, BTN_N1_PIN);
    handleButton(17, BTN_N2_PIN);

    // Build button bitmasks
    lastSent.buttons_low = 0;
    lastSent.buttons_high = 0;

    for (int i = 0; i < 16; i++) {
        if (buttonState[i]) {
            lastSent.buttons_low |= (1 << i);
        }
    }

    for (int i = 16; i < 32; i++) {
        if (buttonState[i]) {
            lastSent.buttons_high |= (1 << (i - 16));
        }
    }

    // Custom buttons M1, M2, N1, N2
    lastSent.buttons_extra = 0;
    if (buttonState[14]) lastSent.buttons_extra |= 0x01; // M1
    if (buttonState[15]) lastSent.buttons_extra |= 0x02; // M2
    if (buttonState[16]) lastSent.buttons_extra |= 0x04; // N1
    if (buttonState[17]) lastSent.buttons_extra |= 0x08; // N2
}

void handleButton(int idx, int pin) {
    uint8_t reading = digitalRead(pin);
    uint32_t now = millis();

    if (reading != lastButtonState[idx]) {
        buttonDebounceTime[idx] = now;
    }

    if ((now - buttonDebounceTime[idx]) > 20) { // Debounce 20ms
        if (reading != buttonState[idx]) {
            buttonState[idx] = reading;
        }
    }

    lastButtonState[idx] = reading;
}

// ==================== SEND CONTROLLER STATE ====================
void sendControllerState() {
    // Pack controller state into frame
    // Format: roll(f32), pitch(f32), yaw(f32), throttle(f32),
    //         buttons_low(u32), buttons_high(u32), buttons_extra(u16)

    uint8_t payload[26];

    // Roll, pitch, yaw, throttle as floats (4 bytes each = 16 bytes)
    float rollF = stickToRaw(lastSent.roll);
    float pitchF = stickToRaw(lastSent.pitch);
    float yawF = stickToRaw(lastSent.yaw);
    float throttleF = stickToRaw(lastSent.throttle);

    memcpy(payload + 0, &rollF, 4);
    memcpy(payload + 4, &pitchF, 4);
    memcpy(payload + 8, &yawF, 4);
    memcpy(payload + 12, &throttleF, 4);

    // Buttons (8 bytes)
    memcpy(payload + 16, &lastSent.buttons_low, 4);
    memcpy(payload + 20, &lastSent.buttons_high, 4);

    // Extra buttons (2 bytes)
    memcpy(payload + 24, &lastSent.buttons_extra, 2);

    // Build full frame (header + payload + dev_id)
    // Frame format: version(1) + type(1) + priority(1) + seq(2) + ts(4) +
    //               payload_len(1) + payload(26) + dev_id(4)

    uint8_t frame[45]; // 10 header + 26 payload + 4 dev_id + buffer

    // Header
    frame[0] = 1;      // version
    frame[1] = LORA_MSG_TYPE;  // CMD_SET_POSITION
    frame[2] = 2;      // priority (NORMAL)
    // Sequence number (2 bytes) - use millis for uniqueness
    uint16_t seq = (uint16_t)(millis() >> 10);
    memcpy(frame + 3, &seq, 2);

    // Timestamp (4 bytes)
    uint32_t ts = millis() / 1000;  // seconds
    memcpy(frame + 5, &ts, 4);

    // Payload length
    frame[9] = 26;

    // Copy payload
    memcpy(frame + 10, payload, 26);

    // Device ID (4 bytes)
    uint32_t devId = DEVICE_ID;
    memcpy(frame + 36, &devId, 4);

    // Transmit
    int state = radio.transmit(frame, 40);  // 10 + 26 + 4 = 40 bytes

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("TX error: ");
        Serial.println(state);
    }
}