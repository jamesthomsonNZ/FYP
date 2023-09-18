#include <Wire.h>
#include <Seeed_HM330X.h>
#include "CFF_ChipCap2.h"
#include "Adafruit_CCS811.h"

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL_OUTPUT SerialUSB
#else
#define SERIAL_OUTPUT Serial
#endif

HM330X hm330x_sensor;
CFF_ChipCap2 cc2_sensor;
Adafruit_CCS811 ccs;

uint8_t hm330x_buf[30];

// const char* str[] = {"sensor num: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
//                      "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
//                      "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
//                      "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
//                      "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
//                      "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
//                     };
const char* str[] = {"sensor num: ", "PM1.0 Standard particulate matter (unit:ug/m3): ",
                     "PM2.5 Standard particulate matter (unit:ug/m3): ",
                     "PM10 Standard particulate matter (unit:ug/m3): ",
                     "PM1.0 Atmospheric environment (unit:ug/m3): ",
                     "PM2.5 Atmospheric environment (unit:ug/m3): ",
                     "PM10 Atmospheric environment (unit:ug/m3): ",
                    };



HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    
    // SERIAL_OUTPUT.print("HM300: ");
    // SERIAL_OUTPUT.print(str);
    // SERIAL_OUTPUT.println(value);

    // Sending data
    //Serial.print("HM300: ");
    //Serial.print(str);
    Serial.print(value);
    Serial.print(",");
    return NO_ERROR;
}

HM330XErrorCode parse_result(uint8_t* data) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 1; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
        print_result(str[i - 1], value);

    }

    return NO_ERROR;
}

HM330XErrorCode parse_result_value(uint8_t* data) {
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 0; i < 28; i++) {
        SERIAL_OUTPUT.print(data[i], HEX);
        SERIAL_OUTPUT.print("  ");
        if ((0 == (i) % 5) || (0 == i)) {
            SERIAL_OUTPUT.println("");
        }
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += data[i];
    }
    if (sum != data[28]) {
        SERIAL_OUTPUT.println("wrong checkSum!!");
    }
    SERIAL_OUTPUT.println("");
    return NO_ERROR;
}

void setup() {
    Serial.begin(115200);
    SERIAL_OUTPUT.begin(115200);
    delay(100);
    SERIAL_OUTPUT.println("Serial start");

    if (hm330x_sensor.init()) {
        SERIAL_OUTPUT.println("HM330X init failed!!");
        while (1);
    }

    cc2_sensor.begin();

    // CCS811 Sensor Setup
    Serial.println("CCS811 test");
    if (!ccs.begin()) {
        Serial.println("Failed to start CCS811 sensor! Please check your wiring.");
        while (1);
    }

    // Wait for the CCS811 sensor to be ready
    while (!ccs.available());
}

void loop() {
    // HM330X Sensor Reading
    if (hm330x_sensor.read_sensor_value(hm330x_buf, 29)) {
        SERIAL_OUTPUT.println("HM330X read result failed!!");
    }
    parse_result(hm330x_buf);
    cc2_sensor.readSensor();


    // ChipCap2 Sensor Reading
    // SERIAL_OUTPUT.println("ChipCap2 Sensor data:");
    // SERIAL_OUTPUT.print("Humidity: ");
    // SERIAL_OUTPUT.print(cc2_sensor.humidity);
    // SERIAL_OUTPUT.println("% RH");
    // SERIAL_OUTPUT.print("Temperature: ");
    // SERIAL_OUTPUT.print(cc2_sensor.temperatureC);
    // SERIAL_OUTPUT.println("°C");
    // SERIAL_OUTPUT.println("");

    //Sending data
    //Serial.print("Humidity: ");
    Serial.print(cc2_sensor.humidity);
    Serial.print(",");
    //Serial.print("% RH,");
    //Serial.print("Temperature: ");
    Serial.print(cc2_sensor.temperatureC);
    //Serial.print("°C,");
    Serial.print(",");

    // CCS811 Sensor Reading
    if (ccs.available()) {
        if (!ccs.readData()) {
            //SERIAL_OUTPUT.print("CO2: ");
            // SERIAL_OUTPUT.print(ccs.geteCO2());
            // SERIAL_OUTPUT.print("ppm, TVOC: ");
            // SERIAL_OUTPUT.print(ccs.getTVOC());
            // SERIAL_OUTPUT.println("ppb");
            
            // Sending data
            //Serial.print("CO2: ");
            Serial.print(ccs.geteCO2());
            Serial.print(",");
            //Serial.print("ppm,TVOC: ");
            Serial.println(ccs.getTVOC());
            //Serial.println("ppb");
        } else {
            Serial.print("CO2: ");
            Serial.println("CCS811 ERROR!");
            while (1);
        }
    }


    delay(5000); // Adjust this delay as needed
}