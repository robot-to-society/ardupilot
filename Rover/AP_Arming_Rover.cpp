#include "AP_Arming_Rover.h"
#include "Rover.h"

// perform pre_arm_rc_checks checks
bool AP_Arming_Rover::rc_calibration_checks(const bool display_failure)
{
    // set rc-checks to success if RC checks are disabled
    if (!check_enabled(Check::RC)) {
        return true;
    }

    const RC_Channel *channels[] = {
        rover.channel_steer,
        rover.channel_throttle
    };
    const char *channel_names[] = {"Steer", "Throttle"};

    for (uint8_t i= 0 ; i < ARRAY_SIZE(channels); i++) {
        const RC_Channel *channel = channels[i];
        const char *channel_name = channel_names[i];
        // check if radio has been calibrated
        if (channel->get_radio_min() > RC_Channel::RC_CALIB_MIN_LIMIT_PWM) {
            check_failed(Check::RC, display_failure, "%s radio min too high", channel_name);
            return false;
        }
        if (channel->get_radio_max() < RC_Channel::RC_CALIB_MAX_LIMIT_PWM) {
            check_failed(Check::RC, display_failure, "%s radio max too low", channel_name);
            return false;
        }
    }
    return AP_Arming::rc_calibration_checks(display_failure);
}

// performs pre_arm gps related checks and returns true if passed
bool AP_Arming_Rover::gps_checks(bool display_failure)
{
    if (!require_location &&
        !rover.control_mode->requires_position() &&
        !rover.control_mode->requires_velocity()) {
        // we don't care!
        return true;
    }

    // call parent gps checks
    if (!AP_Arming::gps_checks(display_failure)) {
        return false;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    // always check if inertial nav has started and is ready
    char failure_msg[50] = {};
    if (!ahrs.pre_arm_check(true, failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "AHRS: %s", failure_msg);
        return false;
    }

    // check for ekf failsafe
    if (rover.failsafe.ekf) {
        check_failed(display_failure, "EKF failsafe");
        return false;
    }

    // ensure position estimate is ok
    if (!rover.ekf_position_ok()) {
        // vehicle level position estimate checks
        check_failed(display_failure, "Need Position Estimate");
        return false;
    }

    return true;
}

bool AP_Arming_Rover::pre_arm_checks(bool report)
{
    if (armed) {
        // if we are already armed then skip the checks
        return true;
    }

    if (!hal.scheduler->is_system_initialized()) {
        check_failed(report, "System not initialised");
        return false;
    }

    // are arming checks disabled?
    if (should_skip_all_checks()) {
        return mandatory_checks(report);
    }

    if (rover.g2.sailboat.sail_enabled() && !rover.g2.windvane.enabled()) {
        check_failed(report, "Sailing enabled with no WindVane");
        return false;
    }

#pragma clang diagnostic push
#if defined(__clang_major__) && __clang_major__ >= 14
#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
#endif
    return (AP_Arming::pre_arm_checks(report)
            & motor_checks(report)
#if AP_OAPATHPLANNER_ENABLED
            & oa_check(report)
#endif
            & parameter_checks(report)
            & mission_arc_checks(report)
            & mode_checks(report));
#pragma clang diagnostic pop
}

bool AP_Arming_Rover::arm_checks(AP_Arming::Method method)
{
    if (method == AP_Arming::Method::RUDDER) {
        // check if arming/disarming allowed from this mode
        if (!rover.control_mode->allows_arming_from_transmitter()) {
            check_failed(true, "Mode not rudder-armable");
            return false;
        }
    }

    // are arming checks disabled?
    if (should_skip_all_checks()) {
        return true;
    }
    return AP_Arming::arm_checks(method);
}

void AP_Arming_Rover::update_soft_armed()
{
    hal.util->set_soft_armed(is_armed());
}

/*
  arm motors
 */
bool AP_Arming_Rover::arm(AP_Arming::Method method, const bool do_arming_checks)
{
    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        return false;
    }

    // Set the SmartRTL home location. If activated, SmartRTL will ultimately try to land at this point
    rover.g2.smart_rtl.set_home(true);

    // initialize simple mode heading
    rover.mode_simple.init_heading();

    // save home heading for use in sail vehicles
    rover.g2.windvane.record_home_heading();

#if HAL_LOGGING_ENABLED
    // Tell logger it can start logging
    AP::logger().set_vehicle_armed(true);
#endif

    update_soft_armed();

    send_arm_disarm_statustext("Throttle armed");

    return true;
}

/*
  disarm motors
 */
bool AP_Arming_Rover::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (method == AP_Arming::Method::RUDDER) {
        if (rover.g2.motors.active()) {
            // can't emit a message here as full-rudder while driving
            // is not uncommon
            return false;
        }
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }
    if (rover.control_mode != &rover.mode_auto) {
        // reset the mission on disarm if we are not in auto
        rover.mode_auto.mission.reset();
    }

#if HAL_LOGGING_ENABLED
    // Tell logger it can stop logging
    AP::logger().set_vehicle_armed(false);
#endif

    update_soft_armed();

    send_arm_disarm_statustext("Throttle disarmed");

    return true;
}

#if AP_OAPATHPLANNER_ENABLED
// check object avoidance has initialised correctly
bool AP_Arming_Rover::oa_check(bool report)
{
    char failure_msg[50] = {};
    if (rover.g2.oa.pre_arm_check(failure_msg, ARRAY_SIZE(failure_msg))) {
        return true;
    }

    check_failed(report, "%s", failure_msg);
    return false;
}
#endif  // AP_OAPATHPLANNER_ENABLED

// perform parameter checks
bool AP_Arming_Rover::parameter_checks(bool report)
{
    // success if parameter checks are disabled
    if (!check_enabled(Check::PARAMETERS)) {
        return true;
    }

    // check waypoint speed is positive
    if (!is_positive(rover.g2.wp_nav.get_default_speed())) {
        check_failed(Check::PARAMETERS, report, "WP_SPEED too low");
        return false;
    }

    return true;
}

// check if arming allowed from this mode
bool AP_Arming_Rover::mode_checks(bool report)
{
    //display failure if arming in this mode is not allowed
    if (!rover.control_mode->allows_arming()) {
        check_failed(report, "Mode not armable");
        return false;
    }
    return true;
}

// check motors are ready
bool AP_Arming_Rover::motor_checks(bool report)
{
    bool ret = rover.g2.motors.pre_arm_check(report);

#if HAL_TORQEEDO_ENABLED
    char failure_msg[50] = {};
    AP_Torqeedo *torqeedo = AP_Torqeedo::get_singleton();
    if (torqeedo != nullptr) {
        if (!torqeedo->pre_arm_checks(failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(report, "Torqeedo: %s", failure_msg);
            ret = false;
        }
    }
#endif

    return ret;
}

// check mission arc waypoint radii against TURN_RADIUS, following the
// mission's execution order (including do-jump commands) so that each
// arc is measured from the location it will actually be entered from
bool AP_Arming_Rover::mission_arc_checks(bool report)
{
    if (!check_enabled(Check::MISSION)) {
        return true;
    }

    const float turn_radius = rover.g2.turn_radius.get();
    if (rover.g2.motors.have_skid_steering() || !is_positive(turn_radius)) {
        return true;
    }

    AP_Mission &mission = rover.mode_auto.mission;
    const uint16_t num_commands = mission.num_commands();
    if (!mission.contains_item(MAV_CMD_NAV_ARC_WAYPOINT)) {
        return true;
    }

    // private copy of AP_Mission's do-jump repeat counters, seeded for
    // a fresh mission start.  The scan below follows jumps exactly as
    // AP_Mission::get_next_cmd does, but never touches the live
    // counters, so the result does not depend on the mission's current
    // run state.
    struct JumpTrack {
        uint16_t index;
        int16_t num_times_run;
    };
    JumpTrack jump_state[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];
    for (uint8_t i = 0; i < AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        jump_state[i].index = AP_MISSION_CMD_INDEX_NONE;
        jump_state[i].num_times_run = 0;
    }

    // resolved do-jump-tag targets, so repeated visits to the same
    // tag do not rescan the mission
    struct TagTrack {
        uint16_t tag;
        uint16_t target;
    };
    TagTrack tag_state[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];
    uint8_t tag_count = 0;

    Location prev_loc = rover.current_loc;
    AP_Mission::Mission_Command cmd;
    uint16_t cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    // bound the total number of commands visited: the cycle
    // detection below terminates the scan of an endless mission as
    // soon as its state repeats, and ordinary missions end by
    // themselves, but nested finite jumps can multiply the visits
    // beyond any bound, as can an endless loop whose state takes
    // long enough to repeat.  If the bound is reached the scan is
    // incomplete, so the check fails (below) rather than pass arcs
    // it never reached
    uint32_t visits_remaining = MIN(512U * num_commands, 10000U);
    bool incomplete = false;
    // get_next_cmd follows at most 64 jump commands in a row before
    // giving up and ending the mission; mirrored here, reset by every
    // non-jump command
    uint8_t jump_chain = 64;
    // advance_current_nav_cmd searches for the next navigation
    // command and gives up after 255 returned commands that did not
    // start one; mirrored here as the count of returned commands
    // since the last started navigation command
    uint16_t search_run = 0;

    // scan state snapshots for cycle detection (Brent's algorithm):
    // the state is everything that decides the scan's future, so if
    // it repeats, every arc entry from here on has already been
    // checked and the scan can stop.  The previous location is part
    // of the state by value, not by command index: a stored command
    // with zero latitude and longitude inherits it, so one index can
    // stand for different locations on different laps
    struct ScanSnapshot {
        uint16_t cmd_index;
        int32_t prev_lat;
        int32_t prev_lng;
        uint8_t jump_chain;
        uint16_t search_run;
        JumpTrack jump_state[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];
    };
    ScanSnapshot snapshot;
    snapshot.cmd_index = cmd_index;
    snapshot.prev_lat = prev_loc.lat;
    snapshot.prev_lng = prev_loc.lng;
    snapshot.jump_chain = jump_chain;
    snapshot.search_run = search_run;
    for (uint8_t i = 0; i < AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        snapshot.jump_state[i] = jump_state[i];
    }
    uint32_t snapshot_window = 1;
    uint32_t steps_since_snapshot = 0;
    // advance the cycle detection by one step; returns true if the
    // scan state has returned to the snapshot state
    auto cycle_check = [&]() -> bool {
        steps_since_snapshot++;
        bool same = (cmd_index == snapshot.cmd_index && prev_loc.lat == snapshot.prev_lat &&
                     prev_loc.lng == snapshot.prev_lng &&
                     jump_chain == snapshot.jump_chain && search_run == snapshot.search_run);
        for (uint8_t i = 0; same && i < AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
            same = (jump_state[i].index == snapshot.jump_state[i].index &&
                    jump_state[i].num_times_run == snapshot.jump_state[i].num_times_run);
        }
        if (same) {
            return true;
        }
        if (steps_since_snapshot >= snapshot_window) {
            snapshot.cmd_index = cmd_index;
            snapshot.prev_lat = prev_loc.lat;
            snapshot.prev_lng = prev_loc.lng;
            snapshot.jump_chain = jump_chain;
            snapshot.search_run = search_run;
            for (uint8_t i = 0; i < AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
                snapshot.jump_state[i] = jump_state[i];
            }
            snapshot_window *= 2;
            steps_since_snapshot = 0;
        }
        return false;
    };

    while (cmd_index < num_commands) {
        if (visits_remaining == 0) {
            incomplete = true;
            break;
        }
        visits_remaining--;
        if (!mission.read_cmd_from_storage(cmd_index, cmd)) {
            break;
        }
        const uint16_t this_index = cmd_index;
        cmd_index++;

        if (cmd.id == MAV_CMD_DO_JUMP_TAG) {
            const uint16_t tag = cmd.content.jump.target;
            cmd.content.jump.target = 0;
            for (uint8_t i = 0; i < tag_count; i++) {
                if (tag_state[i].tag == tag) {
                    cmd.content.jump.target = tag_state[i].target;
                    break;
                }
            }
            if (cmd.content.jump.target == 0) {
                cmd.content.jump.target = mission.get_index_of_jump_tag(tag);
                if (cmd.content.jump.target != 0 && tag_count < AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS) {
                    tag_state[tag_count].tag = tag;
                    tag_state[tag_count].target = cmd.content.jump.target;
                    tag_count++;
                }
            }
            cmd.id = MAV_CMD_DO_JUMP;
        }
        if (cmd.id == MAV_CMD_DO_JUMP) {
            // mirror AP_Mission::get_next_cmd's jump handling
            if (jump_chain-- == 0) {
                // too many jumps in a row: the mission ends there
                break;
            }
            if (cmd.content.jump.target >= num_commands || cmd.content.jump.target == 0) {
                // an invalid target ends the mission there
                break;
            }
            JumpTrack *track = nullptr;
            int16_t times_run = AP_MISSION_JUMP_TIMES_MAX;
            for (uint8_t i = 0; i < AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
                if (jump_state[i].index == this_index) {
                    track = &jump_state[i];
                    times_run = track->num_times_run;
                    break;
                }
                if (jump_state[i].index == AP_MISSION_CMD_INDEX_NONE) {
                    jump_state[i].index = this_index;
                    jump_state[i].num_times_run = 0;
                    track = &jump_state[i];
                    times_run = 0;
                    break;
                }
            }
            if (cmd.content.jump.num_times == AP_MISSION_JUMP_REPEAT_FOREVER || times_run < cmd.content.jump.num_times) {
                if (track != nullptr) {
                    if (cmd.content.jump.num_times == AP_MISSION_JUMP_REPEAT_FOREVER) {
                        // the count never affects a repeat-forever jump;
                        // keep it saturated so scan states can repeat
                        // and be detected below
                        track->num_times_run = 1;
                    } else {
                        track->num_times_run++;
                    }
                }
                cmd_index = cmd.content.jump.target;
            } else if (track != nullptr && !mission.option_is_set(AP_Mission::Option::DONT_ZERO_COUNTER)) {
                // the jump is exhausted; as in get_next_cmd its counter
                // is reset so a later restart of the loop counts again,
                // and execution falls through to the next command
                track->num_times_run = 0;
            }
            if (cycle_check()) {
                break;
            }
            continue;
        }

        // any non-jump command restarts the jump chain budget
        jump_chain = 64;
        // advance_current_nav_cmd ends its search once a navigation
        // command starts; a navigation command the vehicle cannot
        // start (ModeAuto::start_command has no case for it, e.g.
        // NAV_TAKEOFF) leaves the search running and consumes its
        // budget just like a do command does
        bool nav_started = false;
        switch (cmd.id) {
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_ARC_WAYPOINT:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_NAV_LOITER_UNLIM:
        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_NAV_GUIDED_ENABLE:
        case MAV_CMD_NAV_SET_YAW_SPEED:
        case MAV_CMD_NAV_DELAY:
#if AP_SCRIPTING_ENABLED
        case MAV_CMD_NAV_SCRIPT_TIME:
#endif
            nav_started = true;
            break;
        default:
            break;
        }
        if (nav_started) {
            search_run = 0;
        } else if (++search_run >= 255) {
            // as in advance_current_nav_cmd the mission ends there
            break;
        }

        bool is_location_cmd = false;
        switch (cmd.id) {
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_ARC_WAYPOINT:
        case MAV_CMD_NAV_LOITER_UNLIM:
        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_LOITER_TURNS:
            is_location_cmd = true;
            break;
        default:
            break;
        }
        if (is_location_cmd) {
            Location cmdloc = cmd.content.location;
            cmdloc.sanitize(prev_loc);
            if (cmd.id == MAV_CMD_NAV_ARC_WAYPOINT) {
                const float arc_rad = cmd.get_arc_angle_rad();
                const float chord_length_m = prev_loc.get_distance(cmdloc);
                if (is_positive(chord_length_m) && fabsf(wrap_PI(arc_rad)) >= radians(1.0f)) {
                    const float radius_m = fabsf(chord_length_m / (2.0f * fabsf(sinf(arc_rad * 0.5f))));
                    if (radius_m < turn_radius) {
                        check_failed(Check::MISSION, report, "Item %u arc radius %.1f < TURN_RADIUS %.1f", (unsigned)this_index, radius_m, turn_radius);
                        return false;
                    }
                }
            }
            prev_loc = cmdloc;
        }
        if (cycle_check()) {
            break;
        }
    }
    if (incomplete) {
        // ran out of step budget with mission left unscanned
        check_failed(Check::MISSION, report, "Mission too complex to check arcs");
        return false;
    }
    return true;
}
