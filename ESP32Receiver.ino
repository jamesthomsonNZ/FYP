#include <Preferences.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <vector>
#include <map>
#include <Wire.h>

//For PWM signal - Pin configuration
const int pwmPin = 5;  // Pin for PWM output

// LEDC Settings
const int ledcChannel = 0; // LEDC channel for PWM
const int frequency = 5000; // PWM frequency in Hz
const int resolution = 8;   // PWM resolution (8 bits for 0-255)

// Control parameters
const double maxFlowRate = 17.0;  // Max flowrate in L/s
const double maxDutyCycle = 100.0;  // Max duty cycle percentage

// Function that parses the data received from UNO inside and UNO outside
std::vector<String> parseData(const String& data) {
    std::vector<String> dataList;
    String token;
    int start = 0, end = 0;

    while ((end = data.indexOf(',', start)) != -1) {
        token = data.substring(start, end);
        dataList.push_back(token);
        start = end + 1;
    }

    // Add the last token after the last comma
    token = data.substring(start);
    dataList.push_back(token);

    return dataList;
}

// Function to print sensor data for a given sensor
void printSensorData(std::map<String, double>& sensorData, const char* sensorName, const char* variableNames[], size_t numVariables) {
  Serial.println("Sensor Data for " + String(sensorName) + ":");
  
  for (size_t i = 0; i < numVariables; i++) {
    Serial.print(variableNames[i]);
    Serial.print(": ");
    Serial.println(sensorData[variableNames[i]]);
  }
}

// Calculate ACH (Air change per hour)
double calculateACHFromTemperature(double insideTemp, double outsideTemp) {
  double temperatureDifference = insideTemp - outsideTemp;
  double ach = 0.0;

  if (insideTemp >= 18.0 && insideTemp <= 21.0 && outsideTemp < insideTemp) {
    ach = 0.35;
  } else if (insideTemp > 21.0 && insideTemp <= 24.0 && outsideTemp > insideTemp) {
    ach = 0.35;
  } else if (insideTemp > 21 && outsideTemp <= insideTemp && temperatureDifference >= 0.0 && temperatureDifference <= 3.0) {
    ach = (13.0 / 60.0) * temperatureDifference + 0.35;
  } else if (insideTemp < 21 && outsideTemp >= insideTemp && temperatureDifference >= -3.0 && temperatureDifference < 0.0) {
    ach = (-13.0 / 60.0) * temperatureDifference + 0.35;
  } else {
    ach = 0.35;   // Default ACH value if conditions not met
  }
  return ach;
}

// Calculate Duty Cycle 
double calculateDutyCycle(double ach) {
  return ((ach * (1.0 / 3600.0) * 7.2 * (1000.0 / 1.0)) / maxFlowRate) * maxDutyCycle;
}

// Calculate ACH based on HM3001 data sensor
double calculateACHFromVOC_CO2_PM(double voc, double co2, double pm25, double pm10) {
  if (voc > 660.0 || co2 > 5000.0 || pm25 > 35.0 || pm10 > 150.0) {
    return 1.0;
  } else {
    return 0.35;  // Default ACH value if conditions not met
  }
}

// Calculate ACH based from humidity data sensor
double calculateACHFromHumidity(double humidityInside, double humidityOutside) 
{
  if (humidityInside < 30.0 && humidityOutside > humidityInside) {
      return 1.0;
  } else if (humidityInside > 50.0 && humidityInside < 70.0 && humidityOutside < humidityInside) {
    return 0.6;
  } else if (humidityInside >= 70.0 && humidityOutside < humidityInside) {
    return 1.0;
  } else if (humidityInside >= 30.0 && humidityInside <= 50.0) {
    return 0.35;
  } else {
    return 0.35;  // Default ACH value if conditions not met
  }
}

// Function to calculate ACH based on temperature conditions, VOC, CO2, PM and humidity
double calculateACH(std::map<String, double>& sensorInsideData, std::map<String, double>& sensorOutsideData) 
{
  // Calculate ACH based on temperature conditions
  double achFromTemperature = calculateACHFromTemperature(sensorInsideData["in_temperature"], sensorOutsideData["out_temperature"]);
  Serial.print("achFromTemperature: ");
  Serial.println(achFromTemperature);

  // Calculate ACH based on VOC, CO2, and PM conditions
  double achFromVOC_CO2_PM = calculateACHFromVOC_CO2_PM(sensorInsideData["in_TVOC"], sensorInsideData["in_CO2"],
                                                       sensorInsideData["in_pm2.5_a"], sensorInsideData["in_pm10_a"]);
  Serial.print("achFromVOC_CO2_PM: ");
  Serial.println(achFromVOC_CO2_PM);

  // Calculate ACH based on humidity conditions
  double achFromHumidity = calculateACHFromHumidity(sensorInsideData["in_humidity"], sensorOutsideData["out_humidity"]);
  Serial.print("achFromHumidity: ");
  Serial.println(achFromHumidity);

  // Choose the max ACH value from all sources
  double finalACH = max(achFromTemperature, max(achFromVOC_CO2_PM, achFromHumidity));
  Serial.print("finalACH: ");
  Serial.println(finalACH);

  return finalACH;
}


// Function to calculate Duty Cycle and PWM
void calculateDutyCycleAndPWM(double ach) {
  // Calculate duty cycle based on ACH
  double dutyCycle = calculateDutyCycle(ach);

  // Ensure that the duty cycle is within limits
  dutyCycle = constrain(dutyCycle, 0.0, maxDutyCycle);

  // Convert duty cycle to PWM value (0-255)
  int pwmValue = map(dutyCycle, 0, maxDutyCycle, 0, 255);

  // Apply PWM value to the PWM pin using LEDC
  ledcWrite(ledcChannel, pwmValue);

  Serial.print("dutyCycle: ");
  Serial.println(dutyCycle);
  
  Serial.print("pwmValue: ");
  Serial.println(pwmValue);
}


void setup() {
  Serial.begin(115200);  // Initialize Serial Monitor

  // Initialize UART1 on pins 25 (RX) and 26 (TX) for sensorOutsideData
  Serial1.begin(115200, SERIAL_8N1, 25, 26);

  // Initialize ESP32 UART on pins 16 (RX) and 17 (TX) for sensorInsideData
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  //Setup for PWM pin
  pinMode(pwmPin, OUTPUT);

  // Configure LEDC for PWM
  ledcSetup(ledcChannel, frequency, resolution);
  ledcAttachPin(pwmPin, ledcChannel);
}

// Main controller - holds logic for receiving data, parsing data, error handling
//and calculations for  ACH, PWM, and Duty Cycle 
void loop() {

    // Create a map to associate variable names with their values
  std::map<String, double> sensorInsideData;
  std::map<String, double> sensorOutsideData;

  bool errorInData = false; // flag to denote if an error occrus in receving the data

   // Read data from sensorOutsideData (Serial1)
  if (Serial1.available()) 
  {
    String receivedData = Serial1.readStringUntil('\n');
    Serial.print("Received Data from sensorOutsideData: ");
    Serial.println(receivedData);

    // Process the received data by parsing (interpret and extract data from structured format, in this instance separating the "," and extracting the received data)
    std::vector<String> dataList = parseData(receivedData);
    const char* variableNames[] = {"out_sensor_num", "out_pm1_s", "out_pm2.5_s", "out_pm10_s", "out_pm1_a", "out_pm2.5_a", "out_pm10_a", "out_humidity", "out_temperature", "out_CO2", "out_TVOC"};
    size_t dataLength = sizeof(variableNames) / sizeof(variableNames[0]);

    // Check if the number of data received from the sensor is correct (e.g. no missing data). Then, assign corresponding data to variables. If not, set the errorInData flag to true
    // to signify that there's error in the data received.
    if (dataList.size() != dataLength) {
      Serial.println("Error: Invalid number of data values received from sensorOutsideData.");
      errorInData = true;
    } else {
      // Populate the sensorOutsideData with variable names and values
      for (size_t i = 0; i < dataList.size(); i++) {
        double value = dataList[i].toDouble();
        sensorOutsideData[variableNames[i]] = value;
      }
      printSensorData(sensorOutsideData, "Sensor Outside", variableNames, dataLength);
    }

  } else {
    // If no data can be received from Serial1 (sensorOutsideData), set the errorInData flag to true to denote that there's error in the data received.
    Serial.println("Error: Cannot received data from sensorOutsideData.");
    errorInData = true;
  }

  // Read data from sensorInsideData (Serial2)
  if (Serial2.available()) 
  {
    String receivedData = Serial2.readStringUntil('\n');
    Serial.print("Received Data from SensorInsideData: ");
    Serial.println(receivedData);

    // Process the received data
    std::vector<String> dataList = parseData(receivedData);
    const char* variableNames[] = {"in_sensor_num", "in_pm1_s", "in_pm2.5_s", "in_pm10_s", "in_pm1_a", "in_pm2.5_a", "in_pm10_a", "in_humidity", "in_temperature", "in_CO2", "in_TVOC"};
    size_t dataLength = sizeof(variableNames) / sizeof(variableNames[0]);

    // Check if the number of data received from the sensor is correct (e.g. no missing data). Then, assign corresponding data to variables. If not, set the errorInData flag to true
    // to signify that there's error in the data received.    
    if (dataList.size() != dataLength) {
      Serial.println("Error: Invalid number of data values received from SensorInsideData.");
      errorInData = true;
    } else {
      // Populate the map with variable names and values
      for (size_t i = 0; i < dataList.size(); i++) {
        double value = dataList[i].toDouble();
        sensorInsideData[variableNames[i]] = value;
      }
      printSensorData(sensorInsideData, "Sensor Inside", variableNames, dataLength);
    }

  } else {
    // If no data can be received from Serial1 (SensorInsideData), set the errorInData flag to true to denote that there's error in the data received.
    Serial.println("Error: Cannot received data from SensorInsideData.");
    errorInData = true;
  }

  // Dummy parameters
  // sensorInsideData["in_temperature"] = 19.6;
  // sensorOutsideData["out_temperature"] = 16.0;
  // Serial.print("in_temperature: ");
  // Serial.println(sensorInsideData["in_temperature"]);
  // Serial.print("out_temperature: ");
  // Serial.println(sensorOutsideData["out_temperature"]);     


  // If no error then calculate ACH, PWM and Duty Cycle
  if (!errorInData) 
  {
    // Calculate ACH
    double finalACH = calculateACH(sensorInsideData, sensorOutsideData);

    // Calculate Duty Cycle and PWM
    calculateDutyCycleAndPWM(finalACH);
  } else {
    // Set finalACH to 0.35 in case of errors (errorInData = true) either from SensorInsideData or SensorOutsideData
    double finalACH = 0.35;
    calculateDutyCycleAndPWM(finalACH);
  }
  
  delay(5000);
}