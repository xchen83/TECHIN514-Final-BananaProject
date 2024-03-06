#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BLEDevice.h"
#include <Stepper.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// BLE UUIDs
static BLEUUID serviceUUID("74b714af-3163-4663-96c5-da8f52ec7c2a");
static BLEUUID charUUID("5bd0dc14-9942-44da-bb3f-8f5088f20256");


static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

int findMethaneLevel(const String& dataStr) {
    String numStr = "";  // This will store the sequence of digits found
    for (unsigned int i = 0; i < dataStr.length(); i++) {
        if (isDigit(dataStr[i])) {
            numStr += dataStr[i];  // Append the digit to numStr
        } else if (numStr.length() > 0) {
            // If we've already found some digits and then encounter a non-digit,
            // it means we've reached the end of the first number in the string.
            break;
        }
    }
    if (numStr.length() > 0) {
        return numStr.toInt();  // Convert the found digits to an integer
    } else {
        return -1;  // Return -1 or some other value to indicate that no number was found
    }
}


// Function to find the methane level from the data string
static void checkAndActOnMethaneLevel(const String& dataStr) {
    int methaneLevel = findMethaneLevel(dataStr); // Find the methane level from the data string

    // Print the methane level to the Serial monitor
    Serial.print("Received Value for Stepper Motor: "); // Print a label to the Serial monitor
    Serial.println(methaneLevel); // Print the methane level to the Serial monitor

}

bool positionFlag = false;  // Flag for tracking the stepper motor's position
int lastMethaneLevel = -1; // Declare this as a global variable

static void StepperMotorMovement(const String& dataStr) {
    int methaneLevel = findMethaneLevel(dataStr);  // Extract methane level from data string

    // Stepper motor settings

    const int stepsPerRevolution = 200;  // Adjust this to fit the number of steps per revolution for your motor
    const int stepsToPosition = stepsPerRevolution / 2; // Steps to move half a revolution

    Stepper myStepper(stepsPerRevolution, 1, 2, 3, 4);
    
    // Setting up the Stepper motor
    myStepper.setSpeed(60);

    // Print the methane level for debugging purposes
    Serial.print("Stepper Motor Movement Starts!");

    // Act only if there's a change in the methane level
    if (methaneLevel != lastMethaneLevel) {
        lastMethaneLevel = methaneLevel;  // Update the last known methane level

        if (methaneLevel > 1700 && !positionFlag) {
                // Methane level is above 1700 and stepper motor is at Position 1, move to Position 2
                Serial.println("Methane level high, moving to Position 2");
                myStepper.step(stepsPerRevolution / 2);  // Steps to move to Position 2
                positionFlag = true;  // Update flag to indicate the stepper is at Position 2
            } else if (methaneLevel <= 1700 && positionFlag) {
                // Methane level is 1700 or below and stepper motor is at Position 2, move back to Position 1
                Serial.println("Methane level low, moving back to Position 1");
                myStepper.step(-stepsPerRevolution / 2);  // Steps to move back to Position 1
                positionFlag = false;  // Update flag to indicate the stepper is back at Position 1
            } else {
                // Methane level has not crossed the threshold, stepper motor remains in the current position
                Serial.println("Methane level unchanged, stepper motor resting.");
            }
    } else {
        // No change in methane level, no action required
        Serial.println("No change in methane level, no action taken.");
    }

    delay(1000);  // Delay to debounce and allow for motor movement
}

static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic, 
    uint8_t* pData, 
    size_t length, 
    bool isNotify
) {
    // Initialize an empty string to construct the data string
    String dataStr = "";

    // Iterate over each byte of data and append it to the data string
    for (int i = 0; i < length; i++) {
        dataStr += (char)pData[i];  // Cast each byte to a character and append to the string
    }

    // Call the StepperMotorMovement function to process the data string
    checkAndActOnMethaneLevel(dataStr);
    StepperMotorMovement(dataStr);
    
    // Print the processed data string to the Serial monitor
    Serial.print("Processed Data String: ");
    Serial.println(dataStr);

    // Clear the OLED display before displaying new data
    display.clearDisplay();
    display.setTextSize(1);        // Set the text size
    display.setTextColor(SSD1306_WHITE);  // Set the text color
    display.setCursor(0, 0);       // Set the cursor to the top-left corner

    // Display the 'Received Data:' label and the actual data string on the OLED
    display.println(dataStr);
    display.display();  // Update the display with the new data

}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.println("Connected to the server");
    }

    void onDisconnect(BLEClient* pclient) {
        connected = false;
        Serial.println("Disconnected from the server");
    }
};

bool connectToServer() {
    Serial.println("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    BLEClient* pClient = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());
    pClient->connect(myDevice);
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)


    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our service");

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our characteristic");

    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("BLE Advertised Device found: ");
        Serial.println(advertisedDevice.toString().c_str());
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = true;
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Arduino BLE Client application...");
    BLEDevice::init("");

    Wire.begin(5, 6); // Custom I2C pins for OLED

    // Initialize OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Try 0x3D if 0x3C doesn't work
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);  // Don't proceed further
    }

    // Setup BLE and start scanning
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);

    // Test message on OLED to verify it's working
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("BLE Client Start"));
    display.display();
    delay(2000); // Show this message for 2 seconds

}

void loop() {
    if (doConnect == true) {
        if (connectToServer()) {
            Serial.println("Connected to the BLE Server.");
        } else {
            Serial.println("Failed to connect to the server.");
        }
        doConnect = false;
    }

    if (connected) {
        String newValue = "Time since boot: " + String(millis()/1000);
        Serial.println("Setting new characteristic value to \"" + newValue  + "\"");

        // Set the characteristic's value to be the array of bytes that is actually a string.
        pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    }else if(doScan){
        BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
    }

    delay(2000); // Delay a second between loops.
}

