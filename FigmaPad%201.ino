#include <BleKeyboard.h>
#include "AiEsp32RotaryEncoder.h"
#include <Arduino.h>

// BLE Keyboard Device Name
BleKeyboard bleKeyboard("FigmaPad", "ESP32", 100);  

// Define Button Pins & Key Mappings
const int buttonPins[] = {18, 19, 21, 22, 23, 4, 5, 27, 14, 32, 25, 33};
const char keyMappings[] = {'v', 'h', 'r', 'l', 'o', 't', 'f', 'p', '[', ']', '8', '9'};
#define NUM_BUTTONS (sizeof(buttonPins) / sizeof(buttonPins[0]))

// Rotary Encoder Pins
#define ENCODER_CLK 26
#define ENCODER_DT 35
#define ENCODER_SW 15  // Push button (Reset Zoom)

// Use a GPIO that supports OUTPUT and does not affect BLE
#define LED_PIN 13  // Best alternatives: 4, 13, 14, 27, 32

// Rotary Encoder Object
AiEsp32RotaryEncoder rotaryEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, -1);

// Track Button States for Debouncing
bool buttonStates[NUM_BUTTONS] = {false};
unsigned long lastDebounceTime[NUM_BUTTONS] = {0};
const unsigned long debounceDelay = 10;  // 10ms debounce delay

bool wasConnected = false;  // Track BLE Connection State

void IRAM_ATTR readEncoderISR() {
    rotaryEncoder.readEncoder_ISR();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting FigmaPad BLE Keyboard...");

    // Setup LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // LED starts OFF
    Serial.println("LED Pin Configured");

    // BLE Keyboard Initialization
    bleKeyboard.begin();
    Serial.println("BLE Keyboard Initialized");

    // Configure Buttons
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }

    // Configure Rotary Encoder
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    rotaryEncoder.setBoundaries(-10000, 10000, false);
    rotaryEncoder.setAcceleration(5);
    pinMode(ENCODER_SW, INPUT_PULLUP);

    Serial.println("FigmaPad Ready!");
}

void loop() {
    bool isConnected = bleKeyboard.isConnected();

    // Handle BLE Connection State
    if (isConnected && !wasConnected) { // BLE Just Connected
        Serial.println("BLE Connected! Turning LED ON");
        digitalWrite(LED_PIN, HIGH);  // Turn LED ON
    } 
    else if (!isConnected && wasConnected) { // BLE Just Disconnected
        Serial.println("BLE Disconnected! Turning LED OFF");
        digitalWrite(LED_PIN, LOW);  // Turn LED OFF
    }

    wasConnected = isConnected;

    if (isConnected) {
        unsigned long currentTime = millis();

        for (int i = 0; i < NUM_BUTTONS; i++) {
            handleKeyboardButton(buttonPins[i], &buttonStates[i], keyMappings[i], &lastDebounceTime[i], currentTime);
        }

        handleSpecialKeys();
        handleEncoder();
        handleRotaryButton();
    }

    delay(5);  // Minimal delay for fast response
}

// Function to Handle Keyboard Key Press and Release (Optimized with Debouncing)
void handleKeyboardButton(int buttonPin, bool *buttonState, char key, unsigned long *lastTime, unsigned long currentTime) {
    bool pressed = !digitalRead(buttonPin);

    if (pressed != *buttonState && (currentTime - *lastTime) > debounceDelay) {
        *lastTime = currentTime;

        if (pressed) {
            bleKeyboard.press(key);
        } else {
            bleKeyboard.release(key);
        }

        *buttonState = pressed;
    }
}

// Function to handle special shortcut buttons (Optimized)
void handleSpecialKeys() {
    if (!digitalRead(14)) {  // Send Back (CTRL + [)
        bleKeyboard.press(KEY_LEFT_CTRL);
        bleKeyboard.press('[');
        bleKeyboard.releaseAll();
    }
    if (!digitalRead(32)) {  // Bring Forward (CTRL + ])
        bleKeyboard.press(KEY_LEFT_CTRL);
        bleKeyboard.press(']');
        bleKeyboard.releaseAll();
    }
    if (!digitalRead(33)) {  // Open Prototype Panel (ALT + 9)
        bleKeyboard.press(KEY_LEFT_ALT);
        bleKeyboard.press('9');
        bleKeyboard.releaseAll();
    }
    if (!digitalRead(25)) {  // Open Design Panel (ALT + 8)
        bleKeyboard.press(KEY_LEFT_ALT);
        bleKeyboard.press('8');
        bleKeyboard.releaseAll();
    }
}

// Optimized Rotary Encoder Handling (Faster Response)
void handleEncoder() {
    static int lastEncoderValue = 0;
    int encoderValue = rotaryEncoder.readEncoder();

    if (encoderValue != lastEncoderValue) {
        if (encoderValue > lastEncoderValue) {
            bleKeyboard.write('+');  // Zoom In
        } 
        else if (encoderValue < lastEncoderValue) {
            bleKeyboard.write('-');  // Zoom Out
        }
        lastEncoderValue = encoderValue;
    }
}

// Function to handle Rotary Encoder Switch (Reset Zoom)
void handleRotaryButton() {
    static bool lastButtonState = HIGH;
    bool buttonState = digitalRead(ENCODER_SW);

    if (buttonState == LOW && lastButtonState == HIGH) {
        bleKeyboard.press(KEY_LEFT_CTRL);
        bleKeyboard.write('0');
        bleKeyboard.releaseAll();
        delay(20);  // Reduced debounce delay
    }

    lastButtonState = buttonState;
}
