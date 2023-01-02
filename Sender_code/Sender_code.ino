#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>

#define ss 5
#define rst 14
#define dio0 2

#define HEIGHT_TICKS 0

Adafruit_MPU6050 mpu;

TinyGPSPlus gps;
static const int RXPin = 16, TXPin = 17;
SoftwareSerial geo(RXPin, TXPin);

float b_ax, b_ay, b_az, b_gx, b_gy, b_gz;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

float lat0 = 0, lng0 = 0;

boolean fall = false;                       // stores if a fall has occurred
boolean free_fall_flag = false;             // stores if first trigger (lower threshold) has occurred
boolean impact_flag = false;                // stores if second trigger (upper threshold) has occurred
boolean angle_change_flag = false;          // stores if third trigger (orientation change) has occurred
byte free_fall_flag_count = 0;              // stores the counts past since trigger 1 was set true
byte impact_flag_count = 0;                 // stores the counts past since trigger 2 was set true
byte angle_change_count = 0;                // stores the counts past since trigger 3 was set true
int angle_change = 0;

void setup(void)
{
    Serial.begin(115200);
    geo.begin(9600);
    if(!geo){
        Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
        while (1) {
            delay (1000);
        }
    }
    while (!Serial);
    Serial.println("LoRa Sender");

    LoRa.setPins(ss, rst, dio0);
  
    while (!LoRa.begin(433E6))
    {
        Serial.println(".");
        delay(500);
    }
    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Initializing OK!");

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    b_ax = a.acceleration.x;
    b_ay = a.acceleration.y;
    b_az = a.acceleration.z;

    b_gx = g.gyro.x;
    b_gy = g.gyro.y;
    b_gz = g.gyro.z;    

    delay(100);
}

void loop()
{ 
    if (geo.available() > 0) {
        gps.encode(geo.read());
        lat0 = gps.location.lat();
        lng0 = gps.location.lng();
    }
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float mean_square = 0;

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float gx = (g.gyro.x - b_gx)*57.2958;
    float gy = (g.gyro.y - b_gy)*57.2958;
    float gz = (g.gyro.z - b_gz)*57.2958;

    mean_square = pow(ax * ax + ay * ay + az * az, 0.5);
    int Amp = mean_square;
    Serial.println(Amp);

    if (Amp <= 2 && impact_flag == false)
    {
        free_fall_flag = true;
        Serial.println("TRIGGER 1 ACTIVATED");
    }

    if (free_fall_flag == true)
    {
        free_fall_flag_count++;
        if (Amp >= 12 && free_fall_flag_count > HEIGHT_TICKS)
        {
            impact_flag = true;
            Serial.println("TRIGGER 2 ACTIVATED");
            free_fall_flag = false;
            free_fall_flag_count = 0;
        }
    }

    if (impact_flag == true)
    {
        impact_flag_count++;
        angle_change = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
        Serial.print("angle change: ");
        Serial.println(angle_change);
        if (angle_change >= 30 && angle_change <= 400)
        {
            angle_change = true;
            impact_flag = false;
            impact_flag_count = 0;
            Serial.print("angle change: ");
            Serial.println(angle_change);
            Serial.println("TRIGGER 3 ACTIVATED");
        }
    }
    
    if (angle_change == true)
    {
        angle_change_count++;
//        if (angle_change_count >= 0)
//        {
//            angle_change = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
//            Serial.println(angle_change);
//            if ((angle_change >= 0) && (angle_change <= 10))
//            {
                fall = true;
                angle_change = false;
                angle_change_count = 0;
                Serial.print("angle change: ");
                Serial.println(angle_change);
//            }
//            else
//            {
//                angle_change = false;
//                angle_change_count = 0;
//                Serial.println("TRIGGER 3 DE-ACTIVATED");
//            }
//        }
    }
    if (fall == true)
    {
        Serial.println("FALL DETECTED");
        LoRa.beginPacket();
        LoRa.print("1/"+String(lat0,6)+"/"+String(lng0,6));
        Serial.println("1/"+String(lat0,6)+"/"+String(lng0,6));
        LoRa.endPacket();
        fall = false;
    }
    
    if (impact_flag_count >= 6)
    {
        impact_flag = false;
        impact_flag_count = 0;
        Serial.println("TRIGGER 2 DE-ACTIVATED");
    }
    
    if (free_fall_flag && free_fall_flag_count > 0 && Amp > 2)
    {
        free_fall_flag = false;
        free_fall_flag_count = 0;
        Serial.println("TRIGGER 1 DE-ACTIVATED");
    }
    delay(100);
}
