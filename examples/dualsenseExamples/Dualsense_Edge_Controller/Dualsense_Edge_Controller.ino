/*
 * DualSense Edge Controller Example
 *
 * This example emulates a Sony DualSense Edge controller over BLE.
 *
 * IMPORTANT NOTE ON HAPTICS/VIBRATION:
 * Real DualSense controllers do NOT support haptic feedback over Bluetooth on Windows/PC.
 * Sony's haptic system is audio-based and requires a USB connection to expose the controller
 * as an audio device. Over Bluetooth, only basic rumble and adaptive triggers are supported
 * (and support varies by application). This is a hardware/protocol limitation of the
 * DualSense design, not a limitation of this library.
 *
 * The onReceivedOutputReport callback will still fire for any output reports that ARE sent
 * (such as LED color changes), but don't expect haptic/vibration commands from applications
 * like Steam when connected over Bluetooth.
 */

#include <BleConnectionStatus.h>
#include <BleCompositeHID.h>
#include <DualsenseGamepadDevice.h>
#define CONFIG_BT_NIMBLE_EXT_ADV 1

int ledPin = LED_BUILTIN; // LED connected to digital pin 8
uint8_t playerLEDs = 0x00;
uint8_t ledcolor[3] = {0,0,0};
uint8_t muteled = 0;
DualsenseGamepadDevice* dualsense;
BleCompositeHID compositeHID("Libresteishon Edge", "YeaSeb", 100);

// FunctionSlots must be global to persist after setup() completes
FunctionSlot<DualsenseGamepadOutputReportData> vibrationSlot(OnVibrateEvent);
FunctionSlot<DualsenseGamepadOutputReportData> ledSlot(OnLEDEvent);

void OnLEDEvent(DualsenseGamepadOutputReportData data)
{
    Serial.println("LED change:" + data.lightbar_setup);

    if (data.player_leds != playerLEDs) {
        playerLEDs=data.player_leds;
        if ((data.player_leds & DUALSENSE_PLAYERLED_ON) == DUALSENSE_PLAYERLED_ON) {
            if ((data.player_leds & DUALSENSE_PLAYERLED_1) == DUALSENSE_PLAYERLED_1) {
                Serial.println("led 1 on");
            } else {
                Serial.println("led 1 off");
            }
            if ((data.player_leds & DUALSENSE_PLAYERLED_2) == DUALSENSE_PLAYERLED_2) {
                Serial.println("led 2 on");
            } else {
                Serial.println("led 2 off");
            }
            if ((data.player_leds & DUALSENSE_PLAYERLED_3) == DUALSENSE_PLAYERLED_3) {
                Serial.println("led 3 on");
            } else {
                Serial.println("led 3 off");
            }
            if ((data.player_leds & DUALSENSE_PLAYERLED_4) == DUALSENSE_PLAYERLED_4) {
                Serial.println("led 4 on");
            } else {
                Serial.println("led 4 off");
            }
            if ((data.player_leds & DUALSENSE_PLAYERLED_5) == DUALSENSE_PLAYERLED_5) {
                Serial.println("led 5 on");
            } else {
                Serial.println("led 5 off");
            }
        }
    }

    if (data.lightbar_red!=ledcolor[0] || data.lightbar_green!=ledcolor[1] || data.lightbar_blue!=ledcolor[2]){
        ledcolor[0]=data.lightbar_red;
        ledcolor[1]=data.lightbar_green;
        ledcolor[2]=data.lightbar_blue;
        Serial.println(String("LED color change, R: ") + ledcolor[0] + " G: " + ledcolor[1] + " B: " + ledcolor[2]);
    }

    if (data.mute_button_led!=muteled)
    {
        muteled = data.mute_button_led;
        Serial.println(String("Mute button change: ")+muteled);
    }
}

void OnVibrateEvent(DualsenseGamepadOutputReportData data)
{
    if (data.motor_right > 0 || data.motor_left > 0) {
        digitalWrite(ledPin, LOW);
    } else {
        digitalWrite(ledPin, HIGH);
    }
    Serial.println("Output event. Weak motor: " + String(data.motor_left) + " Strong motor: " + String(data.motor_right));
}

void setup()
{
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT); // sets the digital pin as output

    DualsenseEdgeControllerDeviceConfiguration* config = new DualsenseEdgeControllerDeviceConfiguration();
    config->setAutoReport(true);
    config->setAutoDefer(false);

    // The composite HID device pretends to be a valid Dualsense controller via vendor and product IDs (VID/PID).
    // Platforms like windows/linux need this
    BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
    Serial.println("Using VID source: " + String(hostConfig.getVidSource(), HEX));
    Serial.println("Using VID: " + String(hostConfig.getVid(), HEX));
    Serial.println("Using PID: " + String(hostConfig.getPid(), HEX));
    Serial.println("Using GUID version: " + String(hostConfig.getGuidVersion(), HEX));
    Serial.println("Using serial number: " + String(hostConfig.getSerialNumber()));

    // Set up dualsense
    dualsense = new DualsenseGamepadDevice(config);

    // Attach event handlers (FunctionSlots are defined globally)
    dualsense->onReceivedOutputReport.attach(vibrationSlot);
    dualsense->onReceivedOutputReport.attach(ledSlot);

    // Add all child devices to the top-level composite HID device to manage them
    compositeHID.addDevice(dualsense);

    // Start the composite HID device to broadcast HID reports
    // Serial.println("Starting composite HID device...");
    compositeHID.begin(hostConfig);
}

void loop()
{
    if (compositeHID.isConnected()) {
        delay(150);
        dualsense->sendPairingInfoReport();
        dualsense->sendFirmInfoReport();
        dualsense->sendCalibrationReport();

        delay(280);
        dualsense->resetInputs();
        int selection = 32;
        const float STEP = 0.05; // angle change per frame (speed)

        while (compositeHID.isConnected()) {
            Serial.println("Select button/axis to be activated for testing (3 digits in the format 000). 0: Cross 1: Circle  2: Square  3: Triangle \n 4: L1 5: R1 6: L3 7: R3 8: select 9: start 10: Home 11:VolMute \n 12: DPAD u 13: DPAD R 14: DPAD L 15: DPAD D \n 16: L2 17: R2 \n 18:left stick left 19:left stick right 20:left stick down 21:left stick up \n 22:right stick left 23:right stick right 24:right stick down 25:right stick up \n 26: circle joysticks 27: L4 28 : R4 29: L5 30: R5 31: Gyro/Accel 32: Touchpad");
            while (Serial.available() < 3) {
                dualsense->timestamp();
                dualsense->sendGamepadReport();
                delay(20);
            }
            while (Serial.available() > 2) {
                Serial.println("Waiting for input");
                selection = Serial.parseInt();
            }
            Serial.println(selection);
            switch (selection) {
                case 0: {
                    Serial.println("Pressing Cross");
                    dualsense->press(DUALSENSE_BUTTON_A);
                    delay(500);
                    dualsense->release(DUALSENSE_BUTTON_A);
                    break;
                }
                case 1: {
                    dualsense->press(DUALSENSE_BUTTON_B);
                    Serial.println("Pressing Circle");
                    delay(100);
                    dualsense->release(DUALSENSE_BUTTON_B);
                    break;
                }
                case 2: {
                    dualsense->press(DUALSENSE_BUTTON_X);
                    Serial.println("Pressing Square");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_X);
                    break;
                }
                case 3:
                    dualsense->press(DUALSENSE_BUTTON_Y);
                    Serial.println("Pressing Triangle");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_Y);
                    break;
                case 4:
                    dualsense->press(DUALSENSE_BUTTON_LB);
                    Serial.println("Pressing L1");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_LB);
                    break;
                case 5:
                    dualsense->press(DUALSENSE_BUTTON_RB);
                    Serial.println("Pressing R1");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_RB);
                    break;
                case 6:
                    dualsense->press(DUALSENSE_BUTTON_LS);
                    Serial.println("Pressing L3");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_LS);
                    break;
                case 7:
                    dualsense->press(DUALSENSE_BUTTON_RS);
                    Serial.println("Pressing R3");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_RS);
                    break;
                case 8:
                    dualsense->press(DUALSENSE_BUTTON_SELECT);
                    Serial.println("Pressing Select");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_SELECT);
                    break;
                case 9:
                    dualsense->press(DUALSENSE_BUTTON_START);
                    Serial.println("Pressing Start");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_START);
                    break;
                case 10:
                    dualsense->press(DUALSENSE_BUTTON_MODE);
                    Serial.println("Pressing Home");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_MODE);
                    break;
                case 11:
                    dualsense->press(DUALSENSE_BUTTON_MUTE);
                    Serial.println("Pressing Mute");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_MUTE);
                    break;
                case 12:
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_NORTH);
                    Serial.println("Pressing DPAD UP");
                    delay(200);
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_NONE);
                    break;
                case 13:
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_EAST);
                    Serial.println("Pressing DPAD RIGHT");
                    delay(200);
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_NONE);
                    break;
                case 14:
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_WEST);
                    Serial.println("Pressing DPAD LEFT");
                    delay(200);
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_NONE);
                    break;
                case 15:
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_SOUTH);
                    Serial.println("Pressing DPAD DOWN");
                    delay(200);
                    dualsense->pressDPadDirection(DUALSENSE_BUTTON_DPAD_NONE);
                    break;
                case 16:
                    dualsense->setLeftTrigger(100);
                    Serial.println("Pressing L2");
                    delay(200);
                    dualsense->setLeftTrigger(0);
                    delay(200);
                    dualsense->press(DUALSENSE_BUTTON_LT);
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_LT);
                    break;
                case 17:
                    dualsense->setRightTrigger(100);
                    Serial.println("Pressing R2");
                    delay(200);
                    dualsense->setRightTrigger(0);
                    delay(200);
                    dualsense->press(DUALSENSE_BUTTON_RT);
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_RT);
                    break;
                case 18:
                    dualsense->setLeftThumb(-120, 0);
                    Serial.println("Moving left_analog left");
                    delay(200);
                    dualsense->setLeftThumb(0, 0);
                    break;
                case 19:
                    dualsense->setLeftThumb(120, 0);
                    Serial.println("Moving left_analog Right");
                    delay(200);
                    dualsense->setLeftThumb(0, 0);
                    break;
                case 20: {
                    dualsense->setLeftThumb(0, 120);
                    Serial.println("Moving left_analog down");
                    delay(200);
                    dualsense->setLeftThumb(0, 0);
                    break;
                }
                case 21: {
                    dualsense->setLeftThumb(0, -120);
                    Serial.println("Moving left_analog up");
                    delay(200);
                    dualsense->setLeftThumb(0, 0);
                    break;
                }
                case 22: {
                    dualsense->setRightThumb(-120, 0);
                    Serial.println("Moving right_analog left");
                    delay(200);
                    dualsense->setRightThumb(0, 0);
                    break;
                }
                case 23: {
                    dualsense->setRightThumb(120, 0);
                    Serial.println("Moving right_analog Right");
                    delay(200);
                    dualsense->setRightThumb(0, 0);
                    break;
                }
                case 24: {
                    dualsense->setRightThumb(0, 120);
                    Serial.println("Moving right_analog down");
                    delay(200);
                    dualsense->setRightThumb(0, 0);
                    break;
                }
                case 25: {
                    dualsense->setRightThumb(0, -120);
                    Serial.println("Moving right_analog up");
                    delay(200);
                    dualsense->setRightThumb(0, 0);
                    break;
                }
                case 26: {
                    Serial.println("Circle both joysticks");
                    for (float i = 0; i < TWO_PI; i += STEP) {
                        dualsense->setLeftThumb(cos(i) * 100, sin(i) * 100);
                        dualsense->setRightThumb(cos(i) * 100, -sin(i) * 100);
                        delay(15);
                    }
                    dualsense->setRightThumb(0, 0);
                    dualsense->setLeftThumb(0, 0);
                    break;
                }
                case 27: {
                    dualsense->press(DUALSENSE_BUTTON_L4);
                    Serial.println("Pressing L4");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_L4);
                    break;
                }
                case 28: {
                    dualsense->press(DUALSENSE_BUTTON_R4);
                    Serial.println("Pressing R4");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_R4);
                    break;
                }
                case 29: {
                    dualsense->press(DUALSENSE_BUTTON_L5);
                    Serial.println("Pressing L5");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_L5);
                    break;
                }
                case 30: {
                    dualsense->press(DUALSENSE_BUTTON_R5);
                    Serial.println("Pressing R5");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_R5);
                    break;
                }
                case 31: {
                    Serial.println("sending gyro/accel movement");
                    for (float i = 0; i < TWO_PI; i += STEP) {
                        dualsense->setAccel(cos(i) * 300, sin(i) * 300, tan(i) * 300);
                        dualsense->setGyro(cos(i) * 400, sin(i) * 400, tan(i) * 400);
                        delay(5);
                    }
                    break;
                }
                case 32: {
                    Serial.println("Sending touchpad movement");
                    delay(200);
                    for (float i = 0; i < TWO_PI; i += STEP) {
                        dualsense->setLeftTouchpad(cos(i) * 400 + 1000, sin(i) * 400 + 540);
                        delay(15);
                    }
                    delay(20);
                    Serial.println("Sending touchpad click");
                    dualsense->press(DUALSENSE_BUTTON_TOUCHPAD);
                    Serial.println("Pressing R5");
                    delay(200);
                    dualsense->release(DUALSENSE_BUTTON_TOUCHPAD);
                    dualsense->releaseLeftTouchpad();
                    break;
                }
                default: {
                    Serial.println("Selection invalid");
                    Serial.println(selection);
                }
            }
        }

    } else {
        Serial.println("disconnected");
        delay(500);
    }
}
