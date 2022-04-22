#include <Arduino.h>
#include <NeoPixelBus.h>
#include <ESP8266WiFi.h>

const uint8_t updateInterval = 16; // ms
const uint16_t PixelCount = 6; // Only the almighty knows...
const uint8_t PixelPin = 2; // this we mortals might know

NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1800KbpsMethod> strip(PixelCount, PixelPin);
uint8_t serialData[4];
boolean thisIsTrue = false;

void setup() {
    WiFi.mode(WIFI_OFF);
    Serial.begin(115200);
    while (!Serial); // wait for serial attach

    Serial.println();
    Serial.println("Initializing...");
    Serial.flush();

    // this resets all the neopixels to an off state
    strip.Begin();
    strip.Show();

}

RgbColor wheelColor(byte WheelPos, byte iBrightness) {
    float R, G, B;
    float brightness = iBrightness / 255.0;

    if (WheelPos < 85) {
        R = WheelPos * 3;
        G = 255 - WheelPos * 3;
        B = 0;
    } else if (WheelPos < 170) {
        WheelPos -= 85;
        R = 255 - WheelPos * 3;
        G = 0;
        B = WheelPos * 3;
    } else {
        WheelPos -= 170;
        R = 0;
        G = WheelPos * 3;
        B = 255 - WheelPos * 3;
    }
    R = R * brightness + .5;
    G = G * brightness + .5;
    B = B * brightness + .5;
    return RgbColor((byte) R, (byte) G, (byte) B);
}

boolean serialEvent() {
    static uint8_t byteCount = 0;
    while (Serial.available()) {
        boolean inc = true;
        uint8_t inByte = Serial.read();
        if (inByte == 'A') {
            //Serial.print("bytecount: ");
            //Serial.println(byteCount);
            if (byteCount == 0) {
                Serial.write((digitalRead(16) << 3) + (digitalRead(14) << 2) + (digitalRead(12) << 1) +
                             digitalRead(13)); //send ID
                             thisIsTrue = false;
                return false;
            }
            if (byteCount == 4) {
                byteCount = 0;
                Serial.println("returning true");
                thisIsTrue = true;
                return true;
                Serial.println("I didn't return");
            }
        }
        if (byteCount == 4) {
            byteCount = 0;
            inc = false;
        }
        serialData[3 - byteCount] = inByte;
        if (inc) byteCount++;
    }
    thisIsTrue = false;
    return false;
}


void loop() {
    static uint32_t lastUpdate = 0;
/*
    if (millis() >= (lastUpdate + updateInterval)) {
        lastUpdate = millis();
        strip.Show();
    }*/
    if (serialEvent()) {
        Serial.println("got true");
        Serial.println(serialData[3] << 8);
        Serial.println(serialData[2]);
        Serial.println(serialData[1]);
        Serial.println(serialData[0]);
        //strip.SetPixelColor((serialData[3] << 8) + serialData[2], wheelColor(serialData[1], serialData[0]));
    } else {
        if (thisIsTrue) {
            Serial.println("I should never see this");
            thisIsTrue = false;
        }
    }
}