#ifndef LR_HEARTBEAT_HPP
#define LR_HEARTBEAT_HPP

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/failsafe_flags.h>

class MavlinkStreamLRHeartbeat : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamLRHeartbeat(mavlink); }

    static constexpr const char *get_name_static() { return "LR_HEARTBEAT"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_LR_HEARTBEAT; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    bool const_rate() override { return true; }

    unsigned get_size() override
    {
        return MAVLINK_MSG_ID_LR_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    explicit MavlinkStreamLRHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};

    bool send() override
    {
        if (_mavlink->get_free_tx_buf() >= get_size()) {
            vehicle_status_s vehicle_status{};
            _vehicle_status_sub.copy(&vehicle_status);

            vehicle_control_mode_s vehicle_control_mode{};
            _vehicle_control_mode_sub.copy(&vehicle_control_mode);

            actuator_armed_s actuator_armed{};
            _actuator_armed_sub.copy(&actuator_armed);

            vehicle_local_position_s local_pos{};
            _vehicle_local_position_sub.copy(&local_pos);

            vehicle_attitude_s attitude{};
            _vehicle_attitude_sub.copy(&attitude);

            battery_status_s battery{};
            _battery_status_sub.copy(&battery);

            // Initialize message structure
            mavlink_lr_heartbeat_t msg{};

            // Set timestamp
            msg.timestamp = hrt_absolute_time() / 1000;

            // Set basic system info
            msg.type = _mavlink->get_system_type();
            msg.autopilot = MAV_AUTOPILOT_PX4;

            // Set base mode
            uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

            if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
            }

            if (vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON) {
                base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
            }

            if (vehicle_control_mode.flag_control_manual_enabled) {
                base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            }

            if (vehicle_control_mode.flag_control_auto_enabled) {
                base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
            }

            msg.base_mode = base_mode;

            // Set system status
            uint8_t system_status = MAV_STATE_UNINIT;

            if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                system_status = vehicle_status.failsafe ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE;
            } else if (vehicle_status.calibration_enabled || vehicle_status.rc_calibration_in_progress) {
                system_status = MAV_STATE_CALIBRATING;
            } else if (vehicle_status.pre_flight_checks_pass) {
                system_status = MAV_STATE_STANDBY;
            }

            msg.system_status = system_status;

            // Set local position
            msg.x = local_pos.x;
            msg.y = local_pos.y;
            msg.z = local_pos.z;

            // Set heading from attitude
            const matrix::Eulerf euler = matrix::Quatf(attitude.q);
            msg.heading = static_cast<uint8_t>(math::degrees(matrix::wrap_2pi(euler.psi())) * 0.5f);

            // Set battery level
            msg.battery = (battery.connected) ? static_cast<int8_t>(battery.remaining * 100.0f) : -1;

            // Set failure flags
            msg.failure_flags = 0;

            failsafe_flags_s failsafe_flags{};
            if (_failsafe_flags_sub.copy(&failsafe_flags)) {
                if (failsafe_flags.offboard_control_signal_lost) {
                    msg.failure_flags |= HL_FAILURE_FLAG_OFFBOARD_LINK;
                }
                if (failsafe_flags.mission_failure) {
                    msg.failure_flags |= HL_FAILURE_FLAG_MISSION;
                }
                if (failsafe_flags.manual_control_signal_lost) {
                    msg.failure_flags |= HL_FAILURE_FLAG_RC_RECEIVER;
                }
            }

            mavlink_msg_lr_heartbeat_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // LR_HEARTBEAT_HPP
