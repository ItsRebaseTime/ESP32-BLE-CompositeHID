#ifndef DUALSENSE_GAMEPAD_DEVICE_H
#define DUALSENSE_GAMEPAD_DEVICE_H

#include <cstddef>
#include <Callback.h>
#include <NimBLECharacteristic.h>
#include <cstdint>
#include <mutex>

#include "BLEHostConfiguration.h"
#include "BaseCompositeDevice.h"
#include "DualsenseDescriptors.h"
#include "DualsenseGamepadConfiguration.h"
#include "GamepadDevice.h"
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

// DualSense BT Output Report structure (78 bytes total)
// Based on Linux hid-playstation.c dualsense_output_report_bt struct
#pragma pack(push, 1)
struct DualsenseGamepadOutputReportData {
    // BT Header (3 bytes)
    uint8_t report_id = 0;          // Byte 0: Should be 0x31
    uint8_t seq_tag = 0;            // Byte 1: Sequence tag (upper 4 bits = seq, lower 4 = tag)
    uint8_t tag = 0;                // Byte 2: Should be 0x10 (DS_OUTPUT_TAG)

    // Common section starts at byte 3 (47 bytes)
    uint8_t valid_flag0 = 0;        // Byte 3
    uint8_t valid_flag1 = 0;        // Byte 4
    uint8_t motor_right = 0;        // Byte 5: Strong motor (was incorrectly at byte 3)
    uint8_t motor_left = 0;         // Byte 6: Weak motor (was incorrectly at byte 4)

    uint8_t headphone_volume = 0;   // Byte 7
    uint8_t speaker_volume = 0;     // Byte 8
    uint8_t mic_volume = 0;         // Byte 9
    uint8_t audio_control = 0;      // Byte 10
    uint8_t mute_button_led = 0;    // Byte 11

    uint8_t power_save_control = 0; // Byte 12
    uint8_t reserved2[27] = { 0 };  // Bytes 13-39
    uint8_t audio_control2 = 0;     // Byte 40

    uint8_t valid_flag2 = 0;        // Byte 41
    uint8_t reserved3[2] = { 0 };   // Bytes 42-43

    uint8_t lightbar_setup = 0;     // Byte 44
    uint8_t led_brightness = 0;     // Byte 45
    uint8_t player_leds = 0;        // Byte 46
    uint8_t lightbar_red = 0;       // Byte 47
    uint8_t lightbar_green = 0;     // Byte 48
    uint8_t lightbar_blue = 0;      // Byte 49

    uint8_t reserved4[24] = { 0 };  // Bytes 50-73
    uint32_t crc32 = 0;             // Bytes 74-77

    // default constructor OK
    DualsenseGamepadOutputReportData() = default;

    // parsing function - reads from raw BLE output report bytes
    // Handles multiple formats:
    // - 78 bytes: Full BLE report (report_id=0x31 + seq_tag + tag + common)
    // - 77 bytes: USB-over-BLE (seq + common) - DSX sends this format
    //            OR BLE report with report_id stripped (seq_tag + tag + common)
    // - 63 bytes: USB report (report_id=0x02 + common)
    // - 62 bytes: USB report with report_id stripped (common only)
    //
    // DSX 77-byte format: [seq 0x00-0x0F] [valid_flag0] [valid_flag1] [motor_r] [motor_l] ...
    bool load(const uint8_t* value, size_t size)
    {
        if (!value || size < 47)  // Minimum: common section is 47 bytes
            return false;

        int common_offset = 0;  // Offset to start of common section

        if (size >= 77) {
            // 77-78 byte reports - could be BLE or USB-over-BLE format
            if (size >= 78 && value[0] == 0x31) {
                // Full BLE report with report_id 0x31
                report_id = value[0];
                seq_tag = value[1];
                tag = value[2];
                common_offset = 3;
            } else if (value[0] == 0x31) {
                // 77-byte BLE report (report_id present but one byte short)
                report_id = value[0];
                seq_tag = value[1];
                tag = value[2];
                common_offset = 3;
            } else if (value[1] == 0x10) {
                // BLE report with report_id stripped, tag=0x10 at byte 1
                report_id = 0x31;  // Implied
                seq_tag = value[0];
                tag = value[1];
                common_offset = 2;
            } else if (value[0] <= 0x0F) {
                // USB-over-BLE format: first byte is seq counter (0x00-0x0F)
                // Common section starts at byte 1
                // Format: [seq] [valid_flag0] [valid_flag1] [motor_r] [motor_l] ...
                report_id = 0x02;  // USB format
                seq_tag = value[0];
                tag = 0;
                common_offset = 1;
            } else {
                // Unknown format - try assuming common starts at byte 1
                // This handles DSX sending USB-style reports over BLE
                report_id = 0x02;
                seq_tag = value[0];
                tag = 0;
                common_offset = 1;
            }
        } else if (size >= 62) {
            // USB format (62-63 bytes)
            if (size >= 63 && value[0] == 0x02) {
                // Full USB report with report_id
                report_id = value[0];
                seq_tag = 0;
                tag = 0;
                common_offset = 1;
            } else {
                // USB report with report_id stripped
                report_id = 0x02;
                seq_tag = 0;
                tag = 0;
                common_offset = 0;
            }
        } else {
            // Too small - try parsing as common section directly
            report_id = 0;
            seq_tag = 0;
            tag = 0;
            common_offset = 0;
        }

        // Parse common section (47 bytes)
        // Common structure: valid_flag0, valid_flag1, motor_right, motor_left, ...
        valid_flag0 = value[common_offset + 0];
        valid_flag1 = value[common_offset + 1];
        motor_right = value[common_offset + 2];
        motor_left = value[common_offset + 3];

        headphone_volume = value[common_offset + 4];
        speaker_volume = value[common_offset + 5];
        mic_volume = value[common_offset + 6];
        audio_control = value[common_offset + 7];
        mute_button_led = value[common_offset + 8];
        power_save_control = value[common_offset + 9];
        // reserved2[27] at common+10 through common+36
        audio_control2 = value[common_offset + 37];

        valid_flag2 = value[common_offset + 38];
        // reserved3[2] at common+39, common+40
        lightbar_setup = value[common_offset + 41];
        led_brightness = value[common_offset + 42];
        player_leds = value[common_offset + 43];
        lightbar_red = value[common_offset + 44];
        lightbar_green = value[common_offset + 45];
        lightbar_blue = value[common_offset + 46];

        // CRC is at the end of the report (if present)
        if (size >= common_offset + 47 + 4) {
            size_t crc_offset = size - 4;
            crc32 = (uint32_t)value[crc_offset]
                | ((uint32_t)value[crc_offset + 1] << 8)
                | ((uint32_t)value[crc_offset + 2] << 16)
                | ((uint32_t)value[crc_offset + 3] << 24);
        } else {
            crc32 = 0;
        }

        return true;
    }
}__attribute__((packed));
#pragma pack(pop)

static_assert(sizeof(DualsenseGamepadOutputReportData) == DS_OUTPUT_REPORT_BT_SIZE, "Wrong size - expected 78 bytes");

// DualSense BLE Input Report structure (77 bytes, sent after report ID 0x31 prepended by HID layer)
// Byte layout matches `struct dualsense_input_report` in Linux hid-playstation.c
// with a leading BT header byte (bit 0 = HasHID, must be 1; bits 4-7 = seq).
#pragma pack(push, 1)
struct DualsenseGamepadInputReportData {
    uint8_t bt = 0x01;              // byte  0: BLE header (HasHID=1)
    uint8_t x  = 0x80;              // byte  1: left stick X  (ABS_X,  0x80 = centered)
    uint8_t y  = 0x80;              // byte  2: left stick Y  (ABS_Y)
    uint8_t rx = 0x80;              // byte  3: right stick X (ABS_RX, 0x80 = centered)
    uint8_t ry = 0x80;              // byte  4: right stick Y (ABS_RY)
    uint8_t z  = 0;                 // byte  5: L2 analog trigger (ABS_Z)
    uint8_t rz = 0;                 // byte  6: R2 analog trigger (ABS_RZ)
    uint8_t seq = 0x20;             // byte  7: sequence number
    // bytes 8-10: hat (low nibble of byte 8) + 20 buttons spread across the high nibble of
    // byte 8 and all of bytes 9-10. `uint8_t buttons[3]` makes the byte layout deterministic
    // across toolchains (mixing uint8_t/uint32_t bitfields under __attribute__((packed)) is
    // compiler-dependent).
    uint8_t buttons[3] = { 0x08, 0, 0 };  // byte 8 low nibble hat defaults to 0x08 (NONE)
    uint8_t extra_buttons = 0;      // byte 11: Edge paddles / misc
    uint32_t reserved = 0;          // bytes 12-15
    int16_t gyro_x = 0;             // bytes 16-17
    int16_t gyro_y = 0;             // bytes 18-19
    int16_t gyro_z = 0;             // bytes 20-21
    int16_t accel_x = 0;            // bytes 22-23
    int16_t accel_y = 0;            // bytes 24-25
    int16_t accel_z = 0;            // bytes 26-27
    uint32_t timestamp = 0x7621DD40;// bytes 28-31: sensor timestamp
    uint8_t reserved2 = 0;          // byte 32
    uint8_t touchpoint_l_contact = 0x80;  // byte 33 (0x80 = no contact)
    uint16_t touchpoint_l_x : 12;   // bytes 34-35 (low 12 bits across two bytes, packed with y)
    uint16_t touchpoint_l_y : 12;   // byte 36 (high 12 bits)
    uint8_t touchpoint_r_contact = 0x80;  // byte 37
    uint16_t touchpoint_r_x : 12;   // bytes 38-39
    uint16_t touchpoint_r_y : 12;   // byte 40
    uint8_t data_41_53[12];         // bytes 41-52

    // byte 53: battery/status[0]. Low nibble = capacity 0-10, high nibble = charging state.
    // 0x0A = 100% discharging (normal). 0xFF would read as capacity=15 + charging=0xF (error).
    uint8_t status = 0x0A;          // byte 53

    uint8_t data_54_74[18];         // bytes 54-71
    uint8_t reserved3 = 0x00;       // byte 72
    uint32_t crc32 = 0;             // bytes 73-76
} __attribute__((packed));

#pragma pack(pop)

static_assert(sizeof(DualsenseGamepadInputReportData) == 77, "Input report must be 77 bytes (78 on wire after report ID prepended)");

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

    Signal<DualsenseGamepadOutputReportData> onReceivedOutputReport;

    // Input Controls
    void resetInputs();
    void press(uint32_t button = DUALSENSE_BUTTON_A);
    void release(uint32_t button = DUALSENSE_BUTTON_A);
    bool isPressed(uint32_t button = DUALSENSE_BUTTON_A);
    void setLeftThumb(int8_t x = 0, int8_t y = 0);
    void setRightThumb(int8_t x = 0, int8_t y = 0);
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

    // Called by callback when host reads a feature report - populates value before read completes
    void populateFeatureReportOnRead(NimBLECharacteristic* pCharacteristic);

    // Characteristic accessors used by DualsenseGamepadCallbacks to identify
    // which pipe a read/subscribe came from when logging.
    NimBLECharacteristic* getInputChar()          { return getInput(); }
    NimBLECharacteristic* getOutputChar()         { return getOutput(); }
    NimBLECharacteristic* getMinimalInput() const { return _minimalInput; }
    NimBLECharacteristic* getCalibration() const  { return _calibration; }
    NimBLECharacteristic* getFirmwareInfo() const { return _firmwareInfo; }
    NimBLECharacteristic* getPairingInfo() const  { return _pairingInfo; }
    NimBLECharacteristic* getBtPatchInfo() const  { return _btPatchInfo; }

private:
    void sendGamepadReportImpl();
    void sendFirmInfoReportImpl();
    void sendCalibrationReportImpl();
    void sendPairingInfoReportImpl();
    DualsenseGamepadInputReportData _inputReport;
    DualsenseGamepadPairingReportdata _pairingReport;
    void buildFeatureReportWithCrc(uint8_t reportId, const uint8_t* payload,
        size_t payloadSize, uint8_t* outBuffer, size_t outSize);
    NimBLECharacteristic* _extra_input;
    NimBLECharacteristic* _minimalInput;  // 0x01 9-byte Generic Desktop input report (BLE GAP requirement)
    DualsenseGamepadCallbacks* _callbacks;
    DualsenseGamepadDeviceConfiguration* _config;
    NimBLECharacteristic* _calibration;
    NimBLECharacteristic* _firmwareInfo;
    NimBLECharacteristic* _pairingInfo;
    NimBLECharacteristic* _btPatchInfo;
    uint32_t crc32_le(unsigned int crc, unsigned char const* buf, unsigned int len);
    void generate_crc_table(uint32_t* crcTable);
    uint32_t* m_pCrcTable;
    // Threading
    std::mutex _mutex;
};

#endif // DUALSENSE_GAMEPAD_DEVICE_H
