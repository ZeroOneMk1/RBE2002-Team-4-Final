#include "Position_estimation.h"
#include "Encoders.h"

#include <Arduino.h>

/**
 * sendMessage creates a string of the form
 *      topic:message
 * which is what the corresponding ESP32 code expects.
 * */
void Position::sendMessage(const String& topic, const String& message)
{
    Serial1.println(topic + String(':') + message);
}

String serString1;

bool Position::checkSerial1(void)
{
    while(Serial1.available())
    {
        char c = Serial1.read();
        serString1 += c;

        if(c == '\n')
        {
            return true;
        }
    }

    return false;
}

void Position::setup() 
{
    Serial.begin(115200);
    delay(100);  //give it a moment to bring up the Serial

    Serial.println("setup()");

    Serial1.begin(115200);
    digitalWrite(0, HIGH); // Set internal pullup on RX1 to avoid spurious signals

    Serial.println("/setup()");
}

Encoder RomiEncoders;
float x = 0;
float y = 0;
float theta = 0;
unsigned long time_prev = millis();
unsigned long time_now = 0;



void Position::Init(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
    setup();
}

void Position::Stop(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
}

Position::pose_data Position::ReadPose(void)
{
    return {x, y, theta}; // I like this
}

void Position::PrintPose(void)
{
    sendMessage("Position/x", String(x));
    sendMessage("Position/y", String(y));
    sendMessage("Position/theta", String(theta));
    // Serial.print(x);
    // Serial.print('\t');
    // Serial.print(y);
    // Serial.print('\t');
    // Serial.println(theta);
}

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    time_now = millis();
    if (time_now - time_prev >= 50) // update every 50ms for practical reasons
    {
        float interval = (time_now - time_prev) / 1000.0;
        /*
        x += cos(theta) * (target_speed_left + target_speed_right)/2.0 * 50/1000.0;
        y += sin(theta) * (target_speed_left + target_speed_right)/2.0 * 50/1000.0;
        theta += (target_speed_right - target_speed_left)/(target_speed_left + target_speed_right) * 50/1000 * l / 2;
        time_prev = millis();
        */

        // Calculate position for straight movement
        if (target_speed_left == target_speed_right)
        {
            float V = (target_speed_left + target_speed_right) / 2.0;
            x = x + V * cos(theta) * interval;
            y = y + V * sin(theta) * interval;
        }
        else
        { // Calculate position for curved movement
            // find Radius and Angluar Velocity
            float R = (l / 2) * (target_speed_right + target_speed_left) / (target_speed_right - target_speed_left);
            float AngW = (target_speed_right - target_speed_left) / l;
            // find new positions
            x = x - R * sin(theta) + R * sin(theta + AngW * interval);
            y = y + R * cos(theta) - R * cos(theta + AngW * interval);
            theta = theta + AngW * interval;
        }
        time_prev = time_now;
        PrintPose();
    }
}
