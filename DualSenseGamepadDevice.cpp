#include "DualsenseGamepadDevice.h"
#include "BleCompositeHID.h"
#include "DualsenseDescriptors.h"
#include <cstdint>
#include <stdint.h>
#include <string.h>
#include "esp_mac.h"
#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define LOG_TAG "DualsenseGamepadDevice"
#else
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "esp_log_level.h"

static const char* LOG_TAG = "DualsenseGamepadDevice";
#endif

DualsenseGamepadCallbacks::DualsenseGamepadCallbacks(DualsenseGamepadDevice* device)
    : _device(device)
{
}

void DualsenseGamepadCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo)
{

    std::string raw = pCharacteristic->getValue();

    DualsenseGamepadOutputReportData OutputData;
    /*
            for (size_t i = 0; i < raw.length(); i++) {
                Serial.printf(" %d : %02X ", i, (uint8_t)raw[i]);
                if (i % 16 == 0) {
                    Serial.println();
                }
            }
            Serial.println();
    */
    if (!OutputData.load((const uint8_t*)raw.data(), raw.length())) {
        ESP_LOGD(LOG_TAG, "Invalid DS output report size: %d", (int)raw.length());
        return;
    }

    _device->onReceivedOutputReport.fire(OutputData);
}

void DualsenseGamepadCallbacks::onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo)
{
    ESP_LOGD(LOG_TAG, "DualsenseGamepadCallbacks::onRead");
}

void DualsenseGamepadCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue)
{
    ESP_LOGD(LOG_TAG, "DualsenseGamepadCallbacks::onSubscribe, code %d", subValue);
}

void DualsenseGamepadCallbacks::onStatus(NimBLECharacteristic* pCharacteristic, int code)
{
    ESP_LOGD(LOG_TAG, "DualsenseGamepadCallbacks::onStatus, code: %d", code);
}

DualsenseGamepadDevice::DualsenseGamepadDevice()
    : _config(new DualsenseEdgeControllerDeviceConfiguration())
    , _extra_input(nullptr)
    , _callbacks(nullptr)
    , _firmwareInfo(nullptr)
    , _calibration(nullptr)
    , _pairingInfo(nullptr)
{
}

// DualsenseGamepadDevice methods
DualsenseGamepadDevice::DualsenseGamepadDevice(DualsenseGamepadDeviceConfiguration* config)
    : _config(config)
    , _extra_input(nullptr)
    , _callbacks(nullptr)
    , _firmwareInfo(nullptr)
    , _calibration(nullptr)
    , _pairingInfo(nullptr)
{
}

DualsenseGamepadDevice::~DualsenseGamepadDevice()
{
    if (getOutput() && _callbacks) {
        getOutput()->setCallbacks(nullptr);
        delete _callbacks;
        _callbacks = nullptr;
    }

    if (_extra_input) {
        delete _extra_input;
        _extra_input = nullptr;
    }
    if (_firmwareInfo) {
        delete _firmwareInfo;
        _firmwareInfo = nullptr;
    }
    if (_calibration) {
        delete _calibration;
        _calibration = nullptr;
    }
    if (_pairingInfo) {
        delete _pairingInfo;
        _pairingInfo = nullptr;
    }

    if (_config) {
        delete _config;
        _config = nullptr;
    }
}

void DualsenseGamepadDevice::init(NimBLEHIDDevice* hid)
{
    /// Create input characteristic to send events to the computer
    auto input = hid->getInputReport(DUALSENSE_EDGE_INPUT_REPORT_ID);

    // Create output characteristic to handle events coming from the computer
    auto output = hid->getOutputReport(DUALSENSE_EDGE_OUTPUT_REPORT_ID);
    _callbacks = new DualsenseGamepadCallbacks(this);
    output->setCallbacks(_callbacks);
    // pending callbacks for pairing and stuff
    _calibration = hid->getFeatureReport(DUALSENSE_CALIBRATION_REPORT_ID);
    _calibration->setCallbacks(_callbacks);
    _firmwareInfo = hid->getFeatureReport(DUALSENSE_FIRMWARE_INFO_REPORT_ID);
    _firmwareInfo->setCallbacks(_callbacks);
    _pairingInfo = hid->getFeatureReport(DUALSENSE_PAIRING_INFO_REPORT_ID);
    _pairingInfo->setCallbacks(_callbacks);
    // memcpy(_pairingReport.mac_address,hid->esp_bt_dev_get_address();
    setCharacteristics(input, output);
    m_pCrcTable = new uint32_t[256];
    generate_crc_table(m_pCrcTable);
}

const BaseCompositeDeviceConfiguration* DualsenseGamepadDevice::getDeviceConfig() const
{
    // Return the device configuration
    return _config;
}

void DualsenseGamepadDevice::resetInputs()
{
    std::lock_guard<std::mutex> lock(_mutex);
    memset(&_inputReport, 0, sizeof(DualsenseGamepadInputReportData));

    _inputReport.x = DUALSENSE_AXIS_CENTER_OFFSET;
    _inputReport.y = DUALSENSE_AXIS_CENTER_OFFSET;
    _inputReport.z = DUALSENSE_AXIS_CENTER_OFFSET;
    _inputReport.rz = DUALSENSE_AXIS_CENTER_OFFSET;
    _inputReport.hat = 0x00;
    _inputReport.buttons = 0x00;
    _inputReport.touchpoint_l_contact = 0x80;
    _inputReport.touchpoint_r_contact = 0x80;
    _inputReport.battery = 0xFF;
}

void DualsenseGamepadDevice::press(uint32_t button)
{
    // Avoid double presses
    if (!isPressed(button)) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.buttons |= button;
            ESP_LOGD(LOG_TAG, "DualsenseGamepadDevice::press, button: %d", button);
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}
void DualsenseGamepadDevice::setLeftTouchpad(uint16_t x, uint16_t y)
{

    if (_inputReport.touchpoint_l_contact != 0x00) {
        std::lock_guard<std::mutex> lock(_mutex);
        _inputReport.touchpoint_l_contact = 0x00;
    }
    _inputReport.touchpoint_l_x = x & 0xFFF;
    _inputReport.touchpoint_l_y = y & 0xFFF;
    if (_config->getAutoReport()) {
        sendGamepadReport();
    }
}
void DualsenseGamepadDevice::setRightTouchpad(uint16_t x, uint16_t y)
{

    if (_inputReport.touchpoint_r_contact != 0x00) {
        std::lock_guard<std::mutex> lock(_mutex);
        _inputReport.touchpoint_r_contact = 0x00;
    }
    _inputReport.touchpoint_r_x = x & 0xFFF;
    _inputReport.touchpoint_r_y = y & 0xFFF;
    if (_config->getAutoReport()) {
        sendGamepadReport();
    }
}
void DualsenseGamepadDevice::releaseLeftTouchpad()
{
    _inputReport.touchpoint_l_contact = 0x80;
    if (_config->getAutoReport()) {
        sendGamepadReport();
    }
}
void DualsenseGamepadDevice::releaseRightTouchpad()
{
    _inputReport.touchpoint_r_contact = 0x80;
    if (_config->getAutoReport()) {
        sendGamepadReport();
    }
}

void DualsenseGamepadDevice::release(uint32_t button)
{
    // Avoid double presses
    if (isPressed(button)) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.buttons ^= button;
            ESP_LOGD(LOG_TAG, "DualsenseGamepadDevice::release, button: %d", button);
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

bool DualsenseGamepadDevice::isPressed(uint32_t button)
{
    std::lock_guard<std::mutex> lock(_mutex);
    return (bool)((_inputReport.buttons & button) == button);
}

void DualsenseGamepadDevice::setLeftThumb(int8_t x, int8_t y)
{
    x = constrain(x, DUALSENSE_STICK_MIN, DUALSENSE_STICK_MAX);
    y = constrain(y, DUALSENSE_STICK_MIN, DUALSENSE_STICK_MAX);

    if (_inputReport.x != x || _inputReport.y != y) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.x = (uint8_t)(x + DUALSENSE_AXIS_CENTER_OFFSET);
            _inputReport.y = (uint8_t)(y + DUALSENSE_AXIS_CENTER_OFFSET);
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

void DualsenseGamepadDevice::setRightThumb(int8_t z, int8_t rZ)
{
    z = constrain(z, DUALSENSE_STICK_MIN, DUALSENSE_STICK_MAX);
    rZ = constrain(rZ, DUALSENSE_STICK_MIN, DUALSENSE_STICK_MAX);

    if (_inputReport.z != z || _inputReport.rz != rZ) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.z = (uint8_t)(z + DUALSENSE_AXIS_CENTER_OFFSET);
            _inputReport.rz = (uint8_t)(rZ + DUALSENSE_AXIS_CENTER_OFFSET);
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

void DualsenseGamepadDevice::setLeftTrigger(uint8_t value)
{
    value = constrain(value, DUALSENSE_TRIGGER_MIN, DUALSENSE_TRIGGER_MAX);

    if (_inputReport.brake != value) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.brake = value;
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

void DualsenseGamepadDevice::setRightTrigger(uint8_t value)
{
    value = constrain(value, DUALSENSE_TRIGGER_MIN, DUALSENSE_TRIGGER_MAX);

    if (_inputReport.accelerator != value) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.accelerator = value;
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

void DualsenseGamepadDevice::setGyro(int16_t pitch, int16_t yaw, int16_t roll)
{
    pitch = constrain(pitch, -DUALSENSE_GYRO_RANGE, DUALSENSE_GYRO_RANGE);
    yaw = constrain(yaw, -DUALSENSE_GYRO_RANGE, DUALSENSE_GYRO_RANGE);
    roll = constrain(roll, -DUALSENSE_GYRO_RANGE, DUALSENSE_GYRO_RANGE);

    if (_inputReport.gyro_x != pitch || _inputReport.gyro_y != yaw || _inputReport.gyro_z != roll) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.gyro_x = (uint16_t)(pitch + DUALSENSE_AXIS_CENTER_OFFSET);
            _inputReport.gyro_y = (uint16_t)(yaw + DUALSENSE_AXIS_CENTER_OFFSET);
            _inputReport.gyro_z = (uint16_t)(roll + DUALSENSE_AXIS_CENTER_OFFSET);
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}
void DualsenseGamepadDevice::setAccel(int16_t x, int16_t y, int16_t z)
{
    x = constrain(x, -DUALSENSE_ACC_RANGE, DUALSENSE_ACC_RANGE);
    y = constrain(y, -DUALSENSE_ACC_RANGE, DUALSENSE_ACC_RANGE);
    z = constrain(z, -DUALSENSE_ACC_RANGE, DUALSENSE_ACC_RANGE);

    if (_inputReport.accel_x != x || _inputReport.accel_y != y || _inputReport.accel_z != z) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.accel_x = (uint16_t)(x + DUALSENSE_AXIS_CENTER_OFFSET);
            _inputReport.accel_y = (uint16_t)(y + DUALSENSE_AXIS_CENTER_OFFSET);
            _inputReport.accel_z = (uint16_t)(z + DUALSENSE_AXIS_CENTER_OFFSET);
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

void DualsenseGamepadDevice::setTriggers(uint8_t left, uint8_t right)
{
    left = constrain(left, DUALSENSE_TRIGGER_MIN, DUALSENSE_TRIGGER_MAX);
    right = constrain(right, DUALSENSE_TRIGGER_MIN, DUALSENSE_TRIGGER_MAX);

    if (_inputReport.brake != left || _inputReport.accelerator != right) {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.brake = left;
            _inputReport.accelerator = right;
        }
        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

void DualsenseGamepadDevice::pressDPadDirection(uint8_t direction)
{

    // Avoid double presses
    if (!isDPadPressed(direction)) {
        ESP_LOGD(LOG_TAG, "Pressing dpad direction %s", dPadDirectionName(direction).c_str());
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _inputReport.hat = direction;
        }

        if (_config->getAutoReport()) {
            sendGamepadReport();
        }
    }
}

void DualsenseGamepadDevice::pressDPadDirectionFlag(DualsenseDpadFlags direction)
{
    // Filter opposite button presses
    if ((direction & (DualsenseDpadFlags::NORTH | DualsenseDpadFlags::SOUTH)) == (DualsenseDpadFlags::NORTH | DualsenseDpadFlags::SOUTH)) {
        ESP_LOGD(LOG_TAG, "Filtering opposite button presses - up down");
        direction = (DualsenseDpadFlags)(direction ^ (uint8_t)(DualsenseDpadFlags::NORTH | DualsenseDpadFlags::SOUTH));
    }
    if ((direction & (DualsenseDpadFlags::EAST | DualsenseDpadFlags::WEST)) == (DualsenseDpadFlags::EAST | DualsenseDpadFlags::WEST)) {
        ESP_LOGD(LOG_TAG, "Filtering opposite button presses - left right");
        direction = (DualsenseDpadFlags)(direction ^ (uint8_t)(DualsenseDpadFlags::EAST | DualsenseDpadFlags::WEST));
    }

    pressDPadDirection(dPadDirectionToValue(direction));
}

void DualsenseGamepadDevice::releaseDPad()
{
    pressDPadDirection(DUALSENSE_BUTTON_DPAD_NONE);
}

bool DualsenseGamepadDevice::isDPadPressed(uint8_t direction)
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _inputReport.hat == direction;
}

bool DualsenseGamepadDevice::isDPadPressedFlag(DualsenseDpadFlags direction)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (direction == DualsenseDpadFlags::NORTH) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_NORTH;
    } else if (direction == (DualsenseDpadFlags::NORTH | DualsenseDpadFlags::EAST)) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_NORTHEAST;
    } else if (direction == DualsenseDpadFlags::EAST) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_EAST;
    } else if (direction == (DualsenseDpadFlags::SOUTH | DualsenseDpadFlags::EAST)) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_SOUTHEAST;
    } else if (direction == DualsenseDpadFlags::SOUTH) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_SOUTH;
    } else if (direction == (DualsenseDpadFlags::SOUTH | DualsenseDpadFlags::WEST)) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_SOUTHWEST;
    } else if (direction == DualsenseDpadFlags::WEST) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_WEST;
    } else if (direction == (DualsenseDpadFlags::NORTH | DualsenseDpadFlags::WEST)) {
        return _inputReport.hat == DUALSENSE_BUTTON_DPAD_NORTHWEST;
    }
    return false;
}

void DualsenseGamepadDevice::seq()
{
    if (_inputReport.seq < 254) {
        _inputReport.seq += 1;
    } else {
        _inputReport.seq = 0;
    }

    sendGamepadReport();
}
void DualsenseGamepadDevice::timestamp()
{
    uint32_t cycles = ESP.getCycleCount() / 1500;

    _inputReport.timestamp = cycles;
}

void DualsenseGamepadDevice::sendGamepadReport(bool defer)
{
    if (defer || _config->getAutoDefer()) {
        queueDeferredReport(std::bind(&DualsenseGamepadDevice::sendGamepadReportImpl, this));
    } else {
        sendGamepadReportImpl();
    }
}
void DualsenseGamepadDevice::sendFirmInfoReport(bool defer)
{
    if (defer || _config->getAutoDefer()) {
        queueDeferredReport(std::bind(&DualsenseGamepadDevice::sendFirmInfoReportImpl, this));
    } else {
        sendFirmInfoReportImpl();
    }
}
void DualsenseGamepadDevice::sendCalibrationReport(bool defer)
{
    if (defer || _config->getAutoDefer()) {
        queueDeferredReport(std::bind(&DualsenseGamepadDevice::sendCalibrationReportImpl, this));
    } else {
        sendCalibrationReportImpl();
    }
}
void DualsenseGamepadDevice::sendPairingInfoReport(bool defer)
{
    if (defer || _config->getAutoDefer()) {
        queueDeferredReport(std::bind(&DualsenseGamepadDevice::sendPairingInfoReportImpl, this));
    } else {
        sendPairingInfoReportImpl();
    }
}
void DualsenseGamepadDevice::sendGamepadReportImpl()
{
    auto input = getInput();
    auto parentDevice = this->getParent();

    if (!input || !parentDevice)
        return;

    if (!parentDevice->isConnected())
        return;

    {
        std::lock_guard<std::mutex> lock(_mutex);
        size_t packedSize = sizeof(_inputReport);
        ESP_LOGD(LOG_TAG, "Sending gamepad report, size: %d", packedSize);
        uint8_t bthdr[] = { PS_INPUT_CRC32_SEED, DUALSENSE_EDGE_INPUT_REPORT_ID };
        uint32_t crc = 0;
        crc = this->crc32_le(0xFFFFFFFF, (uint8_t*)&bthdr, sizeof(bthdr));
        crc = ~this->crc32_le(crc, (uint8_t*)&_inputReport, sizeof(_inputReport) - 4);
        _inputReport.crc32 = crc;
        ESP_LOGD(LOG_TAG, "got input CRC %lx", crc);
        input->setValue((uint8_t*)&_inputReport, sizeof(_inputReport));
    }
    input->notify();
}
void DualsenseGamepadDevice::sendFirmInfoReportImpl()
{
    auto firminfo = _firmwareInfo;
    auto parentDevice = this->getParent();

    if (!firminfo || !parentDevice)
        return;

    if (!parentDevice->isConnected())
        return;

    {
        std::lock_guard<std::mutex> lock(_mutex);
        size_t packedSize = DUALSENSE_FIRMWARE_INFO_REPORT_SIZE;
        ESP_LOGD(LOG_TAG, "Sending firmware info report, size: %d", packedSize);
        firminfo->setValue((uint8_t*)&DualsenseEdge_FirmwareInfo, packedSize);
    }
    firminfo->indicate();
}
void DualsenseGamepadDevice::sendCalibrationReportImpl()
{
    auto calibration = _calibration;
    auto parentDevice = this->getParent();

    if (!calibration || !parentDevice)
        return;

    if (!parentDevice->isConnected())
        return;

    {
        std::lock_guard<std::mutex> lock(_mutex);
        size_t packedSize = DUALSENSE_CALIBRATION_REPORT_SIZE;
        ESP_LOGD(LOG_TAG, "Sending calibration report, size: %d", packedSize);
        calibration->setValue((uint8_t*)&DualsenseEdge_StockCalibration, packedSize);
    }
    calibration->indicate();
}
void DualsenseGamepadDevice::sendPairingInfoReportImpl()
{
    ESP_LOGD(LOG_TAG, "Sending Pairing report");
    auto pairinginfo = _pairingInfo;
    auto parentDevice = this->getParent();
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    memcpy(_pairingReport.mac_address, mac, 6);
    memcpy(_pairingReport.common, (uint8_t*)&DualsenseEdge_PairInfo_common, 9);
    if (!pairinginfo || !parentDevice)
        return;

    if (!parentDevice->isConnected())
        return;

    {
        std::lock_guard<std::mutex> lock(_mutex);
        uint8_t bthdr[] = { PS_FEATURE_CRC32_SEED, DUALSENSE_PAIRING_INFO_REPORT_ID };
        uint32_t crc = 0;
        crc = this->crc32_le(0xFFFFFFFF, (uint8_t*)&bthdr, sizeof(bthdr));
        crc = ~this->crc32_le(crc, (uint8_t*)&_pairingReport, sizeof(_pairingReport) - 4);
        _pairingReport.crc32 = crc;
        ESP_LOGD(LOG_TAG, "got pairing CRC %lx", crc);
        size_t packedSize = DUALSENSE_PAIRING_INFO_REPORT_SIZE;
        ESP_LOGD(LOG_TAG, "Sending Pair report, size: %d", packedSize);
        pairinginfo->setValue((uint8_t*)&_pairingReport, packedSize);
    }
    pairinginfo->indicate();
}
// taken from https://github.com/StryderUK/BluetoothHID/blob/main/examples/DualShock4
uint32_t DualsenseGamepadDevice::crc32_le(unsigned int crc, unsigned char const* buf, unsigned int len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        crc = m_pCrcTable[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
    }
    return crc;
}
void DualsenseGamepadDevice::generate_crc_table(uint32_t* crcTable)
{
    const uint32_t POLYNOMIAL = 0xEDB88320; // 0x04C11DB7 reversed
    uint32_t remainder;
    uint8_t b = 0;
    do {
        // Start with the data byte
        remainder = b;
        for (unsigned long bit = 8; bit > 0; --bit) {
            if (remainder & 1)
                remainder = (remainder >> 1) ^ POLYNOMIAL;
            else
                remainder = (remainder >> 1);
        }
        crcTable[(size_t)b] = remainder;
    } while (0 != ++b);
}
