#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

#include "USB.h"
#include "USBHIDKeyboard.h"
#include "tusb.h"
#include <Adafruit_NeoPixel.h>

// =========================
// USB Keyboard
// =========================
USBHIDKeyboard Keyboard;

// =========================
// RGB LED for lighting
// =========================
#define RGB_LED_PIN 48

Adafruit_NeoPixel rgb(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastRainbowUpdate = 0;
uint16_t rainbowHue = 0;

void rainbowCycle()
{
    if (millis() - lastRainbowUpdate >= 20)
    {
        lastRainbowUpdate = millis();

        rgb.setPixelColor(
            0,
            rgb.gamma32(rgb.ColorHSV(rainbowHue))
        );

        rgb.show();

        rainbowHue += 500;
    }
}

// =========================
// RC522 Pins
// =========================
#define RFID_SS     14
#define RFID_RST    9

#define RFID_SCK    12
#define RFID_MISO   2
#define RFID_MOSI   10

#define buzzer_pin  40

MFRC522 rfid(RFID_SS, RFID_RST);

// =========================
// RFID health check
// =========================
#define RFID_CHECK_EVERY_MS 2000   // how often to check the chip

unsigned long lastRfidCheck = 0;
uint32_t rfidRestartCount = 0;

// The chip is "alive" only if:
//  1) SPI answers (VersionReg is not 0x00 / 0xFF), and
//  2) the config written by PCD_Init() is still intact.
// (After a glitch / brown-out the RC522 resets its registers: VersionReg still
//  reads fine but the antenna is OFF, so cards are never detected.)
bool rfidAlive()
{
    byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    if (v == 0x00 || v == 0xFF) return false;

    byte tx   = rfid.PCD_ReadRegister(MFRC522::TxControlReg);
    byte ask  = rfid.PCD_ReadRegister(MFRC522::TxASKReg);
    byte mode = rfid.PCD_ReadRegister(MFRC522::ModeReg);

    return ((tx & 0x03) == 0x03) && (ask == 0x40) && (mode == 0x3D);
}

void rfidRestart(const char *why)
{
    rfid.PCD_Init();   // hard reset through the RST pin + full init + antenna ON
    rfidRestartCount++;

    Serial.printf("[RFID] restarted (#%lu) <- %s, version=0x%02X\n",
                  (unsigned long)rfidRestartCount, why,
                  rfid.PCD_ReadRegister(MFRC522::VersionReg));
}

void checkRfidHealth()
{
    if (millis() - lastRfidCheck < RFID_CHECK_EVERY_MS) return;
    lastRfidCheck = millis();

    if (!rfidAlive())
    {
        Serial.println("[RFID] health check FAILED");
        rfidRestart("health check");

        if (!rfidAlive())
        {
            Serial.println("[RFID] still not responding -> check 3.3V / GND / SPI wires");
        }
    }
}

// =========================
// USB HID helper
// =========================
// Types a string. If the PC is suspended/asleep it tries a remote wakeup, and
// if the keyboard still does not answer it logs it and (after repeated failures)
// restarts the ESP32 so USB enumerates again.
uint8_t hidFailCount = 0;

void typeText(const char *text)
{
    if (tud_suspended())
    {
        Serial.println("[USB] bus suspended -> remote wakeup");
        tud_remote_wakeup();
        delay(100);
    }

    bool ready = tud_mounted() && tud_hid_ready();
    size_t sent = ready ? Keyboard.print(text) : 0;
    Keyboard.releaseAll();

    if (sent == strlen(text))
    {
        hidFailCount = 0;
        return;
    }

    hidFailCount++;
    Serial.printf("[USB] HID send FAILED (mounted=%d ready=%d sent=%u, fail #%u)\n",
                  tud_mounted(), ready, (unsigned)sent, hidFailCount);

    if (hidFailCount >= 3)
    {
        Serial.println("[USB] keyboard not responding -> restarting ESP32");
        delay(100);
        ESP.restart();
    }
}

// =========================
// Setup
// =========================
void setup()
{
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);   // never block when no serial monitor is open

    // Buzzer
    pinMode(buzzer_pin, OUTPUT);
    digitalWrite(buzzer_pin, LOW);

    // RGB initialization
    rgb.begin();
    rgb.setBrightness(80);
    rgb.clear();
    rgb.show();

    delay(1000);

    Serial.println();
    Serial.println("Starting RFID USB HID...");

    // -------------------------
    // SPI
    // -------------------------
    SPI.begin(
        RFID_SCK,
        RFID_MISO,
        RFID_MOSI,
        RFID_SS
    );

    // -------------------------
    // RFID
    // -------------------------
    rfid.PCD_Init();

    delay(100);

    Serial.println("RFID initialized.");

    // -------------------------
    // USB HID
    // -------------------------
    Keyboard.begin();
    USB.begin();

    Serial.println("USB Keyboard initialized.");
    Serial.println("System Ready!");
}

// =========================
// Loop
// =========================
void loop()
{
    // Keep RGB changing continuously
    rainbowCycle();

    // Make sure the RC522 is still alive (restart it if not)
    checkRfidHealth();

    // Check if a new card exists
    if (!rfid.PICC_IsNewCardPresent())
    {
        return;
    }

    // Read card
    if (!rfid.PICC_ReadCardSerial())
    {
        return;
    }

    // -------------------------
    // Buzzer
    // -------------------------
    digitalWrite(buzzer_pin, HIGH);
    delay(50);
    digitalWrite(buzzer_pin, LOW);

    // -------------------------
    // Build UID text (e.g. "C0F57A5F")
    // -------------------------
    char uidText[3 * 10 + 1] = "";
    for (byte i = 0; i < rfid.uid.size && i < 10; i++)
    {
        char buffer[3];
        sprintf(buffer, "%02X", rfid.uid.uidByte[i]);
        strcat(uidText, buffer);
    }

    // -------------------------
    // Send UID as Keyboard FIRST (before any Serial print)
    // -------------------------
    unsigned long t0 = millis();
    typeText(uidText);
    unsigned long typingMs = millis() - t0;

    // -------------------------
    // Log (one single print)
    // -------------------------
    Serial.printf("Card UID: %s | typing took %lu ms\n", uidText, typingMs);

    // -------------------------
    // Stop RFID communication
    // -------------------------
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

    // Prevent repeated reading
    delay(1000);
}
