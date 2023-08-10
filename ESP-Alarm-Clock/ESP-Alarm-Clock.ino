/*
 Welcome to Gnd_to_Vcc!!
*/

#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include "segments.h"

#define GMT_OFFSET 3600
#define NUM_LEDS 51
#define PIN_LED D4

#define COLOR_ON  pixels.Color(255,255,255)
#define COLOR_OFF pixels.Color(0,0,0)

// Replace with your network credentials
const char *ssid = "Untrusted Network";
const char *password = "[p@55wort-fu3r5-n3ul@nd]";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

bool led_states[51];

void set_digit(uint8_t digit, uint8_t position, bool show_zero = true)
{
    position %= NUM_OF_DIGITS;
    digit %= 10;
    uint8_t segment_bitcode = numberToSegments[digit];
    uint8_t bitmask = 0b1000000;
    if(digit == 0 && !show_zero)
        bitmask = 0;
    uint8_t offset = position * NUM_OF_SEGMENTS;

    for (int i = 0; i < NUM_OF_SEGMENTS; i++)
    {
        uint8_t led_n = segments[offset + i];
        led_states[led_n] = (segment_bitcode & bitmask) != 0;
        bitmask = bitmask >> 1;
    }
    
}

void set_dots(bool active){
    led_states[SEGMENT_DOT_UPPER] = active;
    led_states[SEGMENT_DOT_LOWER] = active;
}

void set_icons(uint16_t bits){
    led_states[42] = bits & 0b000000001;
    led_states[43] = bits & 0b000000010;
    led_states[44] = bits & 0b000000100;
    led_states[45] = bits & 0b000001000;
    led_states[46] = bits & 0b000010000;
    led_states[47] = bits & 0b000100000;
    led_states[48] = bits & 0b001000000;
    led_states[49] = bits & 0b010000000;
    led_states[50] = bits & 0b100000000;
}

void write_leds(uint32_t color_on, uint32_t color_off){
    for (int i = 0; i < NUM_LEDS; i++)
    {
        if(led_states[i])
            pixels.setPixelColor(i, color_on);
        else
            pixels.setPixelColor(i, color_off);
    }
    pixels.show();
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Connect to Wi-Fi
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("connecting..");
    }

    timeClient.begin();
    timeClient.setTimeOffset(GMT_OFFSET);

    pixels.begin();

    set_digit(8, 0);
    set_digit(8, 1);
    set_digit(8, 2);
    set_digit(8, 3);
    write_leds(COLOR_ON, COLOR_OFF);

    delay(10000);
}

void loop()
{
    timeClient.update();
    set_dots(millis() % 2000 > 1000);

    set_digit(timeClient.getHours() / 10, 0, false);
    set_digit(timeClient.getHours(), 1);

    set_digit(timeClient.getMinutes() / 10, 2);
    set_digit(timeClient.getMinutes(), 3);

    set_icons(0b110010101);

    //write_leds(Wheel(millis() / 20), COLOR_OFF);
    write_leds(pixels.Color(255,255,255), COLOR_OFF);
}
