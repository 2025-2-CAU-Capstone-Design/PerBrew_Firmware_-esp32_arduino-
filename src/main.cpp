#include <Arduino.h>
#include "../driver/pin_map.h"
#include "../driver/grinder/grinder_driver.h"
#include "../driver/loadcell/loadcell_driver.h"
#include "../driver/heater/heater_driver.h"
#include "../driver/pouring/pouringSection_driver.h"

// 드라이버 인스턴스들
GrinderDriver grinder;
LoadCellDriver loadcell;
heaterDriver heater;
pouringSectionDriver pouring;

// put function declarations here:
void testModules();
void testGrindingSection();
void testHeaterSection();
void testLoadcellSection();
void testPouringSection();
int waitForInput();
void printMainMenu();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10); // 시리얼 준비 대기
  
  Serial.println("PerBrew Dev");
  Serial.println("Module testing modes...");
  
  // 드라이버들 초기화
  Serial.println("Initializing drivers...");
  grinder.begin();
  loadcell.begin();
  heater.begin();
  pouring.begin();
  Serial.println("All drivers initialized!");
  
  testModules();
}

void loop() {
}

// put function definitions here:
void testModules() {
  while(true) {
    printMainMenu();
    int choice = waitForInput();
    
    switch(choice) {
      case 1:
        testGrindingSection();
        break;
      case 2:
        testHeaterSection();
        break;
      case 3:
        testLoadcellSection();
        break;
      case 4:
        testPouringSection();
        break;
      case 0:
        Serial.println("Exiting test mode...");
        return;
      default:
        Serial.println("Invalid choice! Please try again.");
        break;
    }
    Serial.println("\n" + String("=").substring(0, 40));
  }
}

void printMainMenu() {
  Serial.println("\n=== PerBrew Module Test Menu ===");
  Serial.println("Select module to test:");
  Serial.println("1. Grinding Section");
  Serial.println("2. Heater Section");
  Serial.println("3. LoadCell");
  Serial.println("4. Pouring Section");
  Serial.println("0. Exit");
  Serial.print("Enter choice (0-4): ");
}

int waitForInput() {
  while (!Serial.available()) {
    delay(50);
  }
  int input = Serial.parseInt();
  Serial.println(input); // 입력한 값 표시
  // 버퍼 비우기
  while (Serial.available()) {
    Serial.read();
  }
  return input;
}

void testGrindingSection() {
  Serial.println("\n=== Grinding Section Test ===");
  
  while(true) {
    Serial.println("\nGrinding Test Menu:");
    Serial.println("1. Read current clicks");
    Serial.println("2. Convert ADC to clicks");
    Serial.println("3. Set clicks position");
    Serial.println("4. Calibrate (home position)");
    Serial.println("5. Grind test");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    
    int choice = waitForInput();
    
    switch(choice) {
      case 1: {
        int clicks = grinder.getCurrentClicks();
        Serial.print("Current clicks: ");
        Serial.println(clicks);
        break;
      }
      case 2: {
        int clicks = grinder.convertADCtoClicks();
        Serial.print("ADC converted to clicks: ");
        Serial.println(clicks);
        break;
      }
      case 3: {
        Serial.print("Enter target clicks (0-240): ");
        int target = waitForInput();
        Serial.print("Setting clicks to: ");
        Serial.println(target);
        grinder.setClicks(target);
        Serial.print("New position: ");
        Serial.println(grinder.getCurrentClicks());
        break;
      }
      case 4: {
        Serial.println("Starting calibration...");
        bool success = grinder.setBaseCalibration();
        Serial.print("Calibration ");
        Serial.println(success ? "SUCCESS" : "FAILED");
        if (success) {
          Serial.print("Home position set. Current clicks: ");
          Serial.println(grinder.getCurrentClicks());
        }
        break;
      }
      case 5: {
        Serial.println("Starting grind test...");
        grinder.grind();
        Serial.println("Grind function executed");
        break;
      }
      case 0:
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  }
}

void testHeaterSection() {
  Serial.println("\n=== Heater Section Test ===");
  
  while(true) {
    Serial.println("\nHeater Test Menu:");
    Serial.println("1. Read current temperature");
    Serial.println("2. Set target temperature");
    Serial.println("3. Update PID control");
    Serial.println("4. Continuous monitoring (10 seconds)");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    
    int choice = waitForInput();
    
    switch(choice) {
      case 1: {
        double temp = heater.readThermistor();
        Serial.print("Raw temperature reading: ");
        if (isnan(temp)) {
          Serial.println("ERROR - Invalid reading");
        } else {
          Serial.print(temp);
          Serial.println(" °C");
        }
        Serial.print("Current filtered temperature: ");
        Serial.print(heater.getCurrentTemperature());
        Serial.println(" °C");
        break;
      }
      case 2: {
        Serial.print("Enter target temperature (°C): ");
        double target = (double)waitForInput();
        heater.setTargetTemperature(target);
        Serial.print("Target temperature set to: ");
        Serial.print(target);
        Serial.println(" °C");
        break;
      }
      case 3: {
        heater.updateValue();
        Serial.println("PID control updated");
        Serial.print("Current temperature: ");
        Serial.print(heater.getCurrentTemperature());
        Serial.println(" °C");
        break;
      }
      case 4: {
        Serial.println("Monitoring for 10 seconds...");
        unsigned long startTime = millis();
        while (millis() - startTime < 10000) {
          heater.updateValue();
          Serial.print("Temp: ");
          Serial.print(heater.getCurrentTemperature());
          Serial.println(" °C");
          delay(1000);
        }
        break;
      }
      case 0:
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  }
}

void testLoadcellSection() {
  Serial.println("\n=== LoadCell Section Test ===");
  
  while(true) {
    Serial.println("\nLoadCell Test Menu:");
    Serial.println("1. Check if ready");
    Serial.println("2. Read current weight");
    Serial.println("3. Tare (zero calibration)");
    Serial.println("4. Set scale factor");
    Serial.println("5. Calibrate with known mass");
    Serial.println("6. Update weight (blocking)");
    Serial.println("7. Try update weight (timeout)");
    Serial.println("8. Continuous monitoring (10 seconds)");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    
    int choice = waitForInput();
    
    switch(choice) {
      case 1: {
        bool ready = loadcell.isReady();
        Serial.print("LoadCell ready: ");
        Serial.println(ready ? "YES" : "NO");
        break;
      }
      case 2: {
        float weight = loadcell.getWeight();
        Serial.print("Current weight: ");
        Serial.println(weight);
        break;
      }
      case 3: {
        Serial.println("Performing tare...");
        bool success = loadcell.tare();
        Serial.print("Tare ");
        Serial.println(success ? "SUCCESS" : "FAILED");
        Serial.print("New offset: ");
        Serial.println(loadcell.getOffset());
        break;
      }
      case 4: {
        Serial.print("Enter scale factor: ");
        float scale = (float)waitForInput();
        loadcell.setScale(scale);
        Serial.print("Scale factor set to: ");
        Serial.println(loadcell.getScale());
        break;
      }
      case 5: {
        Serial.print("Enter known mass (g): ");
        float mass = (float)waitForInput();
        Serial.println("Place known mass and performing calibration...");
        delay(2000);
        bool success = loadcell.calibrate(mass);
        Serial.print("Calibration ");
        Serial.println(success ? "SUCCESS" : "FAILED");
        Serial.print("New scale: ");
        Serial.println(loadcell.getScale());
        break;
      }
      case 6: {
        Serial.println("Updating weight (blocking)...");
        loadcell.updateWeightBlocking();
        Serial.print("Weight: ");
        Serial.println(loadcell.getWeight());
        break;
      }
      case 7: {
        Serial.print("Enter timeout (ms): ");
        uint32_t timeout = (uint32_t)waitForInput();
        Serial.println("Trying to update weight...");
        bool success = loadcell.tryUpdateWeight(timeout);
        Serial.print("Update ");
        Serial.println(success ? "SUCCESS" : "TIMEOUT");
        if (success) {
          Serial.print("Weight: ");
          Serial.println(loadcell.getWeight());
        }
        break;
      }
      case 8: {
        Serial.println("Monitoring for 10 seconds...");
        unsigned long startTime = millis();
        while (millis() - startTime < 10000) {
          if (loadcell.isReady()) {
            loadcell.updateWeightBlocking(1);
            Serial.print("Weight: ");
            Serial.println(loadcell.getWeight());
          } else {
            Serial.println("Not ready");
          }
          delay(500);
        }
        break;
      }
      case 0:
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  }
}

void testPouringSection() {
  Serial.println("\n=== Pouring Section Test ===");
  
  while(true) {
    Serial.println("\nPouring Test Menu:");
    Serial.println("1. Measure distance");
    Serial.println("2. Rotate nozzle test");
    Serial.println("3. Tilt nozzle test");
    Serial.println("4. Get last duration");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    
    int choice = waitForInput();
    
    switch(choice) {
      case 1: {
        long distance = pouring.measureDistanceCM();
        Serial.print("Distance: ");
        if (distance == -1) {
          Serial.println("No object detected (timeout)");
        } else {
          Serial.print(distance);
          Serial.println(" mm");
        }
        break;
      }
      case 2: {
        Serial.print("Enter steps: ");
        int steps = waitForInput();
        Serial.print("Enter direction (0=CCW, 1=CW): ");
        bool direction = (bool)waitForInput();
        Serial.println("Rotating nozzle...");
        pouring.rotateNozzleTest(steps, direction);
        Serial.println("Rotation completed");
        break;
      }
      case 3: {
        Serial.print("Enter steps: ");
        int steps = waitForInput();
        Serial.print("Enter direction (0=down, 1=up): ");
        bool direction = (bool)waitForInput();
        Serial.println("Tilting nozzle...");
        pouring.tiltNozzleTest(steps, direction);
        Serial.println("Tilt completed");
        break;
      }
      case 4: {
        int duration = pouring.getDuration();
        Serial.print("Last pulse duration: ");
        Serial.print(duration);
        Serial.println(" microseconds");
        break;
      }
      case 0:
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  }
}