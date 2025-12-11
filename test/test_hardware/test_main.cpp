#include <Arduino.h>
#include "../../driver/pin_map.h"
#include "../../driver/grinder/grinder_driver.h"
#include "../../driver/loadcell/loadcell_driver.h"
#include "../../driver/heater/heater_driver.h"
#include "../../driver/pouring/pouringSection_driver.h"
#include "../../driver/arranging/arranging_driver.h"

/*
요청 (클라이언트 -> 서버)
{
  "recipe_id": 1
}

서버에서 ESP32로 전송하는 데이터
{
  "type": "RECIPE",
  "payload": {
    "recipe_id": 1,
    "recipe_name": "Handsome Wade V60 Recipe",
    "dose_g": 15.0,
    "water_temperature_c": 94.0,
    "total_water_g": 235.0,
    "total_brew_time_s": 180,
    "brew_ratio": 15.67,
    "grind_level": "90",
    "grind_microns": 631,
    "rinsing": true,
    "pouring_steps": [
      {
        "step_number": 1,
        "water_g": 30.0,
        "pour_time_s": 15.0,
        "wait_time_s": 40.0,
        "bloom_time_s": null,
        "technique": "center"
      },
      {
        "step_number": 2,
        "water_g": 70.0,
        "pour_time_s": 20.0,
        "wait_time_s": 0.0,
        "bloom_time_s": null,
        "technique": "spiral_out"
      },
      {
        "step_number": 3,
        "water_g": 70.0,
        "pour_time_s": 20.0,
        "wait_time_s": 0.0,
        "bloom_time_s": null,
        "technique": "spiral_out"
      },
      {
        "step_number": 4,
        "water_g": 65.0,
        "pour_time_s": 15.0,
        "wait_time_s": 0.0,
        "bloom_time_s": null,
        "technique": "spiral_out"
      }
    ]
  }
}


*/

// 드라이버 인스턴스들
GrinderDriver grinder;
LoadCellDriver loadcell;
HeaterDriver heater;
PouringSectionDriver pouring;
ArrangingDriver arranging;

// put function declarations here:
void testModules();
void testGrindingSection();
void testHeaterSection();
void testLoadcellSection();
void testPouringSection();
void testArrangingSection();
int waitForInput();
void printMainMenu();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10); // 시리얼 준비 대기
  
  Serial.println("PerBrew Dev - Updated Test");
  Serial.println("Module testing modes...");
  
  // 드라이버들 초기화
  Serial.println("Initializing drivers...");
  grinder.begin();
  loadcell.begin();
  heater.begin();
  pouring.begin();
  arranging.begin();
  Serial.println("All drivers initialized!");
  
  testModules();
}

void loop() {
    // 테스트 모드는 setup() 내의 testModules() 루프에서 실행되므로 loop()는 비워둡니다.
}

// put function definitions here:
void printMainMenu() {
  Serial.println("\n=== PerBrew Module Test Menu ===");
  Serial.println("Select module to test:");
  Serial.println("1. Grinding Section");
  Serial.println("2. Heater Section");
  Serial.println("3. LoadCell");
  Serial.println("4. Pouring Section");
  Serial.println("5. Arranging Section");
  Serial.println("0. Exit");
  Serial.print("Enter choice (0-5): ");
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
      case 5:
        testArrangingSection();
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

// =================================================================================
// 1. Grinding Section
// =================================================================================
void testGrindingSection() {
  Serial.println("\n=== Grinding Section Test ===");
  
  while(true) {
    Serial.println("\nGrinding Test Menu:");
    Serial.println("1. Read current clicks");
    Serial.println("2. Set clicks position");
    Serial.println("3. Start Grinding (Manual Stop)");
    Serial.println("4. Stop Grinding");
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
        Serial.print("Enter target clicks (0-240): ");
        int target = waitForInput();
        Serial.print("Setting clicks to: ");
        Serial.println(target);
        grinder.setClicks(target);
        Serial.print("New position: ");
        Serial.println(grinder.getCurrentClicks());
        break;
      }
      case 3: {
        Serial.println("Starting grinding motor...");
        grinder.startGrinding();
        Serial.println("Motor ON. Press '4' to stop in menu.");
        break;
      }
      case 4: {
        Serial.println("Stopping grinding motor...");
        grinder.stopGrinding();
        Serial.println("Motor OFF.");
        break;
      }
      case 0:
        grinder.stopGrinding(); // 안전을 위해 나갈 때 정지
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  }
}
void emergencyStop() {
  Serial.println("\n[SAFETY] Emergency Stop - Shutting down all drivers...");
  grinder.stopGrinding();
  heater.stopHeating();
  pouring.stopPump();
  pouring.stopRotation();
  Serial.println("[SAFETY] All drivers stopped.");
}

// =================================================================================
// 2. Heater Section
// =================================================================================
void testHeaterSection() {
  Serial.println("\n=== Heater Section Test ===");
  
  while(true) {
    Serial.println("\nHeater Test Menu:");
    Serial.println("1. Read current temperature");
    Serial.println("2. Start Heating (Target Temp)");
    Serial.println("3. Stop Heating");
    Serial.println("4. Manual Update Loop (Monitor & Control)");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    emergencyStop();
    int choice = waitForInput();
    
    switch(choice) {
      case 1: {
        double temp = heater.readThermistor();
        Serial.print("Raw Temp: "); Serial.println(temp);
        Serial.print("Filtered Temp: "); Serial.println(heater.getCurrentTemperature());
        break;
      }
      case 2: {
        Serial.print("Enter target temperature (°C): ");
        double target = (double)waitForInput();
        heater.startHeating(target);
        Serial.print("Heating started. Target: ");
        Serial.println(target);
        break;
      }
      case 3: {
        heater.stopHeating();
        Serial.println("Heating stopped.");
        break;
      }
      case 4: {
        Serial.println("Monitoring... Press any key to stop.");
        // 버퍼 비우기
        while(Serial.available()) Serial.read();
        
        while (!Serial.available()) {
          heater.update(); // PID 계산 및 제어
          Serial.print("Temp: ");
          Serial.print(heater.getCurrentTemperature());
          Serial.println(" °C");
          delay(500);
        }
        // 입력된 키 소비
        while(Serial.available()) Serial.read();
        break;
      }
      case 0:
        emergencyStop();
        heater.stopHeating();
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  }
}

// =================================================================================
// 3. LoadCell Section
// =================================================================================
void testLoadcellSection() {
  Serial.println("\n=== LoadCell Section Test ===");
  
  while(true) {
    Serial.println("\nLoadCell Test Menu:");
    Serial.println("1. Read current weight");
    Serial.println("2. Tare (10 samples)");
    Serial.println("3. Calibrate with known mass");
    Serial.println("4. Continuous monitoring");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    
    int choice = waitForInput();
    
    switch(choice) {
      case 1: {
        loadcell.updateWeightBlocking();
        float weight = loadcell.getWeight();
        Serial.print("Current weight: ");
        Serial.println(weight);
        break;
      }
      case 2: {
        Serial.println("Performing tare (10 samples)...");
        bool success = loadcell.tare(); 
        Serial.print("Tare ");
        Serial.println(success ? "SUCCESS" : "FAILED");
        break;
      }
      case 3: {
        Serial.print("Enter known mass (g): ");
        float mass = (float)waitForInput();
        Serial.println("Place known mass...");
        delay(2000);
        bool success = loadcell.calibrate(mass);
        Serial.println(success ? "Calibration SUCCESS" : "Calibration FAILED");
        break;
      }
      case 4: {
        Serial.println("Monitoring... Press any key to stop.");
        while(Serial.available()) Serial.read();
        
        while (!Serial.available()) {
          if (loadcell.isReady()) {
            loadcell.updateWeightBlocking();
            Serial.print("Weight: ");
            Serial.println(loadcell.getWeight());
          }
          delay(200);
        }
        while(Serial.available()) Serial.read();
        break;
      }
      case 0:
        emergencyStop();
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  }
}

// =================================================================================
// 4. Pouring Section
// =================================================================================
void testPouringSection() {
  Serial.println("\n=== Pouring Section Test ===");
  
  while(true) {
    Serial.println("\nPouring Test Menu:");
    Serial.println("2. Start Pump");
    Serial.println("3. Stop Pump");
    Serial.println("4. Tilt Nozzle (Auto by Distance)");
    Serial.println("5. Start Rotation (DC Motor)");
    Serial.println("6. Stop Rotation");
    Serial.println("7. Tilt Nozzle (Angle)");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    
    int choice = waitForInput();
    
    switch(choice) {
      case 2: {
        Serial.print("Enter PWM (0-255): ");
        int pwm = waitForInput();
        pouring.startPump(pwm);
        Serial.println("Pump Started.");
        break;
      }
      case 3: {
        pouring.stopPump();
        Serial.println("Pump Stopped.");
        break;
      }
      case 4: {
        Serial.print("Enter target distance (cm) to simulate: ");
        long dist = (long)waitForInput();
        Serial.println("Tilting nozzle...");
        pouring.tiltToEndStop();
        Serial.println("Done.");
        break;
      }
      case 5: {
        Serial.print("Enter PWM (0-255): ");
        int pwm = waitForInput();
        Serial.print("Direction (0:CCW, 1:CW): ");
        int dir = waitForInput();
        pouring.startRotation(dir == 1, pwm);
        Serial.println("Rotation Started.");
        break;
      }
      case 6: {
        pouring.stopRotation();
        Serial.println("Rotation Stopped.");
        break;
      }
      case 7: {
        Serial.print("Enter angle (degrees): ");
        float angle = (float)waitForInput();
        Serial.println("Tilt Done.");
        break;
      }
      case 0:
        emergencyStop();
        pouring.stopPump();
        pouring.stopRotation();
        return;
      default:
        Serial.println("Invalid choice!");
        break;
    }
    pouring.update(); 
  }
}

// =================================================================================
// 5. Arranging Section
// =================================================================================
void testArrangingSection() {
  Serial.println("\n=== Arranging Section Test ===");
  
  while(true) {
    Serial.println("\nArranging Test Menu:");
    Serial.println("1. Move Steps");
    Serial.println("0. Back to main menu");
    Serial.print("Enter choice: ");
    
    int choice = waitForInput();
    
    switch(choice) {
      case 1: {
        Serial.print("Enter steps (positive/negative): ");
        int steps = waitForInput();
        Serial.print("Moving "); Serial.print(steps); Serial.println(" steps...");
        arranging.move(steps);
        Serial.println("Move complete.");
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
