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

void write_leds(){
    for (int i = 0; i < NUM_LEDS; i++)
    {
        if(led_states[i])
            pixels.setPixelColor(i, COLOR_ON);
        else
            pixels.setPixelColor(i, COLOR_OFF);
    }
    pixels.show();
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
    
    set_digit(1, 0);
    set_digit(2, 1);

    set_digit(3, 2);
    set_digit(4, 3);

    write_leds();
    delay(2000);
}

void loop()
{
    timeClient.update();
    set_dots(millis() % 2000 > 1000);

    set_digit(timeClient.getHours() / 10, 0, false);
    set_digit(timeClient.getHours(), 1);

    set_digit(timeClient.getMinutes() / 10, 2);
    set_digit(timeClient.getMinutes(), 3);

    write_leds();
}
