#ifndef DUALSENSE_GAMEPAD_DEVICE_H
#define DUALSENSE_GAMEPAD_DEVICE_H

#include <Callback.h>
#include <NimBLECharacteristic.h>
#include <cstdint>
#include <mutex>

#include "BLEHostConfiguration.h"
#include "BaseCompositeDevice.h"
#include "DualsenseDescriptors.h"
#include "DualsenseGamepadConfiguration.h"
#include "GamepadDevice.h"
#include "esp_mac.h"
// Button bitmasks
#define DUALSENSE_BUTTON_Y 0x08
#define DUALSENSE_BUTTON_B 0x04
#define DUALSENSE_BUTTON_A 0x02
#define DUALSENSE_BUTTON_X 0x01

#define DUALSENSE_BUTTON_DPAD_NONE 0x08
#define DUALSENSE_BUTTON_DPAD_NORTH 0x00
#define DUALSENSE_BUTTON_DPAD_NORTHEAST 0x01
#define DUALSENSE_BUTTON_DPAD_EAST 0x02
#define DUALSENSE_BUTTON_DPAD_SOUTHEAST 0x03
#define DUALSENSE_BUTTON_DPAD_SOUTH 0x04
#define DUALSENSE_BUTTON_DPAD_SOUTHWEST 0x05
#define DUALSENSE_BUTTON_DPAD_WEST 0x06
#define DUALSENSE_BUTTON_DPAD_NORTHWEST 0x07
#define DUALSENSE_BUTTON_LB 0x10
#define DUALSENSE_BUTTON_RB 0x20
#define DUALSENSE_BUTTON_LT 0x40
#define DUALSENSE_BUTTON_RT 0x80
#define DUALSENSE_BUTTON_SELECT 0x100
#define DUALSENSE_BUTTON_START 0x200
#define DUALSENSE_BUTTON_LS 0x400
#define DUALSENSE_BUTTON_RS 0x800

#define DUALSENSE_BUTTON_MODE 0x1000
#define DUALSENSE_BUTTON_TOUCHPAD 0x2000
#define DUALSENSE_BUTTON_SHARE 0x4000
#define DUALSENSE_BUTTON_MUTE 0x8000
#define DUALSENSE_BUTTON_L4 0x10000
#define DUALSENSE_BUTTON_R4 0x20000
#define DUALSENSE_BUTTON_L5 0x40000
#define DUALSENSE_BUTTON_R5 0x80000

// Dpad values

// Dpad bitflags
enum DualsenseDpadFlags : uint8_t {
    NONE = 0x08,
    NORTH = 0x00,
    EAST = 0x02,
    SOUTH = 0x04,
    WEST = 0x08
};

// Trigger range
#define DUALSENSE_TRIGGER_MIN 0
#define DUALSENSE_TRIGGER_MAX 255

// Thumbstick range
#define DUALSENSE_STICK_MIN -127
#define DUALSENSE_STICK_MAX 127
#define DUALSENSE_ACC_RES_PER_G 8192
#define DUALSENSE_ACC_RANGE (4 * 8192)
#define DUALSENSE_GYRO_RES_PER_DEG_S 1024
#define DUALSENSE_GYRO_RANGE (2048 * 1024)
#define DUALSENSE_AXIS_CENTER_OFFSET 0x80

// player leds masks
#define DUALSENSE_PLAYERLED_ON 0x20
#define DUALSENSE_PLAYERLED_1 0x01
#define DUALSENSE_PLAYERLED_2 0x02
#define DUALSENSE_PLAYERLED_3 0x04
#define DUALSENSE_PLAYERLED_4 0x08
#define DUALSENSE_PLAYERLED_5 0x10

// Forwards
class DualsenseGamepadDevice;

class DualsenseGamepadCallbacks : public NimBLECharacteristicCallbacks {
public:
    DualsenseGamepadCallbacks(DualsenseGamepadDevice* device);

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override;

private:
    DualsenseGamepadDevice* _device;
};
struct DualsenseGamepadOutputReportData {
    uint8_t type;
    uint8_t seq_tag = 0;
    uint8_t tag = 0;
    uint8_t motor_right = 0, motor_left = 0;

    uint8_t headphone_volume = 0, speaker_volume = 0, mic_volume = 0;
    uint8_t audio_control = 0;
    uint8_t mute_button_led = 0;

    uint8_t valid_flag0 = 0, valid_flag1 = 0;
    uint8_t power_save_control = 0;
    uint8_t reserved0[27] = { 0 };
    uint8_t audio_control2 = 0;

    uint8_t reserved1[2] = { 0 };
    uint8_t valid_flag2 = 0;
    uint8_t lightbar_setup = 0;
    uint8_t player_leds = 0;
    uint8_t lightbar_red = 0;
    uint8_t lightbar_green = 0;
    uint8_t lightbar_blue = 0;

    uint8_t reserved2[24] = { 0 };
    uint32_t crc32 = 0;

    // default constructor OK
    DualsenseGamepadOutputReportData() = default;

    // parsing function
    bool load(const uint8_t* value, size_t size)
    {
        if (!value || size < 77)
            return false;
        type = value[0];
        seq_tag = value[1];
        tag = value[2];
        motor_right = value[3];
        motor_left = value[4];

        headphone_volume = value[5];
        speaker_volume = value[6];
        mic_volume = value[7];
        audio_control = value[8];
        mute_button_led = value[9];
        valid_flag0 = value[10];
        valid_flag1 = value[11];
        power_save_control = value[12];
        audio_control2 = value[39];

        valid_flag2 = value[42];
        lightbar_setup = value[43];
        player_leds = value[44];
        lightbar_red = value[45];
        lightbar_green = value[46];
        lightbar_blue = value[47];

        crc32 = (uint32_t)value[73]
            | ((uint32_t)value[74] << 8)
            | ((uint32_t)value[75] << 16)
            | ((uint32_t)value[76] << 24);

        return true;
    }
} __attribute__((packed));

static_assert(sizeof(DualsenseGamepadOutputReportData) == 77, "Wrong size");

#pragma pack(push, 1)
struct DualsenseGamepadInputReportData {
    uint8_t bt = 0x10; // -1
    uint8_t x = 0x80; // 0 Left joystick X
    uint8_t y = 0x80; // 1 Left joystick Y
    uint8_t z = 0x80; // 2 Right jostick X
    uint8_t rz = 0x80; // 3 Right joystick Y
    uint8_t brake = 0; // 4 8 bits for brake (left trigger)
    uint8_t accelerator = 0; // 5 8 bits for accelerator (right trigger)
    uint8_t seq = 0x20; // 6
    uint8_t hat : 4; // 6.5 4bits for hat switch (Dpad) + 0 bit padding (0.5 byte)
    uint32_t buttons : 20; // 9 20 * 1bit for buttons + 8 bit padding (4 bytes)
    uint8_t extra_buttons; // 10
    uint32_t reserved; // 14
    uint16_t gyro_x = 0; // 16
    uint16_t gyro_y = 0; // 18
    uint16_t gyro_z = 0; // 20
    uint16_t accel_x = 0; // 22
    uint16_t accel_y = 0; // 24
    uint16_t accel_z = 0; // 26
    uint32_t timestamp = 0x7621DD40; // 30
    uint8_t reserved2 = 0; // 31
    uint8_t touchpoint_l_contact = 0;
    uint16_t touchpoint_l_x : 12; // 32 - 34
    uint16_t touchpoint_l_y : 12; // 34.5 - 35
    uint8_t touchpoint_r_contact = 0; // 36
    uint16_t touchpoint_r_x : 12; // 32 - 34
    uint16_t touchpoint_r_y : 12; // 34.5 - 35
    uint8_t data_41_53[12];
    uint8_t battery = 0xFF;
    uint8_t data_54_74[18];
    uint8_t reserved3 = 0x00;

    uint32_t crc32 = 0; // 4-byte CRC32 checksum
} __attribute__((packed));
#pragma pack(pop)
struct DualsenseGamepadPairingReportdata {
    uint8_t mac_address[6];
    uint8_t common[9];
    uint32_t crc32;
} __attribute__((packed));

static uint8_t dPadDirectionToValue(DualsenseDpadFlags direction)
{
    if (direction == DualsenseDpadFlags::NORTH)
        return DUALSENSE_BUTTON_DPAD_NORTH;
    else if (direction == (DualsenseDpadFlags::EAST | DualsenseDpadFlags::NORTH))
        return DUALSENSE_BUTTON_DPAD_NORTHEAST;
    else if (direction == DualsenseDpadFlags::EAST)
        return DUALSENSE_BUTTON_DPAD_EAST;
    else if (direction == (DualsenseDpadFlags::EAST | DualsenseDpadFlags::SOUTH))
        return DUALSENSE_BUTTON_DPAD_SOUTHEAST;
    else if (direction == DualsenseDpadFlags::SOUTH)
        return DUALSENSE_BUTTON_DPAD_SOUTH;
    else if (direction == (DualsenseDpadFlags::WEST | DualsenseDpadFlags::SOUTH))
        return DUALSENSE_BUTTON_DPAD_SOUTHWEST;
    else if (direction == DualsenseDpadFlags::WEST)
        return DUALSENSE_BUTTON_DPAD_WEST;
    else if (direction == (DualsenseDpadFlags::WEST | DualsenseDpadFlags::NORTH))
        return DUALSENSE_BUTTON_DPAD_NORTHWEST;

    return DUALSENSE_BUTTON_DPAD_NONE;
}

static String dPadDirectionName(uint8_t direction)
{
    if (direction == DUALSENSE_BUTTON_DPAD_NORTH)
        return "NORTH";
    else if (direction == DUALSENSE_BUTTON_DPAD_NORTHEAST)
        return "NORTHEAST";
    else if (direction == DUALSENSE_BUTTON_DPAD_EAST)
        return "EAST";
    else if (direction == DUALSENSE_BUTTON_DPAD_SOUTHEAST)
        return "SOUTHEAST";
    else if (direction == DUALSENSE_BUTTON_DPAD_SOUTH)
        return "SOUTH";
    else if (direction == DUALSENSE_BUTTON_DPAD_SOUTHWEST)
        return "SOUTHWEST";
    else if (direction == DUALSENSE_BUTTON_DPAD_WEST)
        return "WEST";
    else if (direction == DUALSENSE_BUTTON_DPAD_NORTHWEST)
        return "NORTHWEST";
    return "NONE";
}

class DualsenseGamepadDevice : public BaseCompositeDevice {
public:
    DualsenseGamepadDevice();
    DualsenseGamepadDevice(DualsenseGamepadDeviceConfiguration* config);
    ~DualsenseGamepadDevice();

    void init(NimBLEHIDDevice* hid) override;
    const BaseCompositeDeviceConfiguration* getDeviceConfig() const override;

    Signal<DualsenseGamepadOutputReportData> onVibrate;

    // Input Controls
    void resetInputs();
    void press(uint32_t button = DUALSENSE_BUTTON_A);
    void release(uint32_t button = DUALSENSE_BUTTON_A);
    bool isPressed(uint32_t button = DUALSENSE_BUTTON_A);
    void setLeftThumb(int8_t x = 0, int8_t y = 0);
    void setRightThumb(int8_t z = 0, int8_t rZ = 0);
    void setLeftTrigger(uint8_t rX = 0);
    void setRightTrigger(uint8_t rY = 0);
    void setTriggers(uint8_t rX = 0, uint8_t rY = 0);
    void pressDPadDirection(uint8_t direction = 0);
    void pressDPadDirectionFlag(DualsenseDpadFlags direction = DualsenseDpadFlags::NONE);
    void releaseDPad();
    bool isDPadPressed(uint8_t direction = 0);
    bool isDPadPressedFlag(DualsenseDpadFlags direction);
    void pressShare();
    void releaseShare();
    void setLeftTouchpad(uint16_t x, uint16_t y);
    void setRightTouchpad(uint16_t x, uint16_t y);
    void releaseLeftTouchpad();
    void releaseRightTouchpad();
    void setAccel(int16_t x, int16_t y, int16_t z);
    void setGyro(int16_t pitch, int16_t yaw, int16_t roll);
    void sendGamepadReport(bool defer = false);
    void sendFirmInfoReport(bool defer = false);
    void sendCalibrationReport(bool defer = false);
    void sendPairingInfoReport(bool defer = false);
    void timestamp();
    void seq();

private:
    void sendGamepadReportImpl();
    void sendFirmInfoReportImpl();
    void sendCalibrationReportImpl();
    void sendPairingInfoReportImpl();
    DualsenseGamepadInputReportData _inputReport;
    DualsenseGamepadPairingReportdata _pairingReport;
    void signCRC32Inplace(const uint8_t* data, uint8_t seed);
    NimBLECharacteristic* _extra_input;
    DualsenseGamepadCallbacks* _callbacks;
    DualsenseGamepadDeviceConfiguration* _config;
    NimBLECharacteristic* _calibration;
    NimBLECharacteristic* _firmwareInfo;
    NimBLECharacteristic* _pairingInfo;
    uint32_t crc32_le(unsigned int crc, unsigned char const* buf, unsigned int len);
    void generate_crc_table(uint32_t* crcTable);
    uint32_t* m_pCrcTable;
    // Threading
    std::mutex _mutex;
};

#endif // DUALSENSE_GAMEPAD_DEVICE_H
