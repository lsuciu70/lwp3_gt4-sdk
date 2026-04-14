/**
 * @file Lwp3Constants.hpp
 * @author lsuciu
 * @brief Internal protocol constants and Type-Safe Enums for LWP3.
 */

#pragma once
#include <cstdint>
#include <string_view>

namespace LWP3 {

// --- Bluetooth Hardware Identifiers ---
static constexpr std::string_view SERVICE_UUID = "00001623-1212-efde-1623-785feabcd123";
static constexpr std::string_view CHAR_UUID = "00001624-1212-efde-1623-785feabcd123";

// --- Porsche GT4 Physical Port Mapping ---
static constexpr uint8_t PORT_STEER = 0x34;
static constexpr uint8_t PORT_DRIVE_L = 0x32;
static constexpr uint8_t PORT_DRIVE_R = 0x33;

/** @brief LWP3 Packet Lengths for common commands. */
enum class MessageLength : uint8_t {
    HUB_PROP_SETUP = 0x05,
    VIRTUAL_PORT_SETUP = 0x06,
    MOTOR_POWER = 0x09,
    PORT_INPUT_FORMAT = 0x0A,
    GOTO_ABS_POS = 0x0E
};

/** @brief LWP3 Message Type Identifiers (Type-Safe). */
enum class MessageType : uint8_t {
    HUB_PROPERTIES = 0x01,
    HUB_ATTACHED_IO = 0x04,
    GENERIC_ERROR_MESSAGES = 0x05,
    PORT_INPUT_FORMAT_SETUP = 0x41,
    PORT_VALUE_SINGLE = 0x45,
    PORT_OUTPUT_COMMAND = 0x81,
    VIRTUAL_PORT_SETUP = 0x61
};

/** @brief Hub Property Identifiers. */
enum class HubProperty : uint8_t { BUTTON_STATE = 0x02, RSSI = 0x05, BATTERY_VOLTAGE = 0x06 };

/** @brief Property operation modes. */
enum class HubPropertyOperation : uint8_t { ENABLE_UPDATES = 0x02, UPDATE = 0x06 };

/** @brief Sub-commands for port control. */
enum class PortOutputSubCommand : uint8_t { START_POWER = 0x07, GOTO_ABS_POS = 0x0D };
}  // namespace LWP3