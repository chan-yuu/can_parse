#pragma once

#include <cstdint>
#include <memory>

#define CHASSIS_LIB_COMMON_NAMESPACE_BEGIN namespace chassis { namespace common {
#define CHASSIS_LIB_COMMON_NAMESPACE_END }}

CHASSIS_LIB_COMMON_NAMESPACE_BEGIN

struct ControlCommand {
    double timestamp;
    float accelerator;
    float brake;
    float target_velocity;
    float target_steering_angle;
    uint8_t target_gear;
    uint8_t turn_lights;
    uint8_t head_lights;
    uint8_t horn_chassis;
    uint8_t fog_lamp_front;
    uint8_t fog_lamp_rear;
    uint8_t wiper_chassis;

    ControlCommand() { reset(); }

    void reset() {
        timestamp = 0.0;
        accelerator = 0.0;
        brake = 0.0;
        target_velocity = 0.0;
        target_steering_angle = 0.0;
        target_gear = 0;
        turn_lights = 0;
        head_lights = 0;
        horn_chassis = 0;
        fog_lamp_front = 0;
        fog_lamp_rear = 0;
        wiper_chassis = 0;
    }

    bool isValid() const { // Added 'const'
        if (timestamp < 0.0) return false;
        if (target_gear > 3) return false;
        return true;
    }

    typedef std::shared_ptr<ControlCommand> Ptr;
    typedef std::shared_ptr<const ControlCommand> ConstPtr;
};

CHASSIS_LIB_COMMON_NAMESPACE_END