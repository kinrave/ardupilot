#include "mode.h"
#include "Plane.h"

/*
  mode LANDN parameters
 */
const AP_Param::GroupInfo ModeLandn::var_info[] = {
    // @Param: XT_ACCPTBL
    // @DisplayName: Acceptable crosstrack error
    // @Description: Acceptable crosstrack error to initiate the maneuver
    // @Range: 0 10
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("XT_ACCPTBL", 1, ModeLandn, xt_acceptable, 4),

    // @Param: BNK_ACCPTB
    // @DisplayName: Acceptable bank
    // @Description: Acceptable absolute bank while in LANDN mode
    // @Range: 0 30
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("BNK_ACCPTB", 2, ModeLandn, bank_acceptable, 10),

    // @Param: DT_MOT_CUT
    // @DisplayName: Time befor Maneuver to cut motors
    // @Description: Time in seconds before initiating the LANDN dive to cut off the motor
    // @Units: s
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DT_MOT_CUT", 3, ModeLandn, dt_motor_cut, 0.1),

    // @Param: INIT_DIST
    // @DisplayName: Distance to init LANDN
    // @Description: The distance to the last WP to init the LANDN dive.
    // @Units: m
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("INIT_DIST", 4, ModeLandn, landn_init_dist, 10),

    // @Param: RATE
    // @DisplayName: Pitch angle rate
    // @Description: Pitch angle rate to reach target angle
    // @Units: deg/s
    // @Range: -100 -10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE", 5, ModeLandn, landn_rate, -20),

    // @Param: TARGET_CD
    // @DisplayName: Target angle
    // @Description: Target angle for LANDN mode.
    // @Units: cdeg
    // @Range: -8900 -1000
    // @User: Advanced
    AP_GROUPINFO("TARGET_CD", 6, ModeLandn, landn_target_cd, -4500),
    
    AP_GROUPEND
};

ModeLandn::ModeLandn()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeLandn::_enter()
{
    plane.landn_state.motors_off = false;
    plane.landn_state.diving = false;
    plane.landn_state.locked_roll = false;
    plane.landn_state.locked_pitch = false;
    plane.landn_state.landn_rate = constrain_float(landn_rate, -270, -10);
    plane.landn_state.landn_target_cd = constrain_int32(landn_target_cd, -8900, -1000);

    gcs().send_text(MAV_SEVERITY_CRITICAL,"LANDN: entered, target %i, rate %f",int(landn_target_cd),float(landn_rate));

    return true;
}

void ModeLandn::update()
{
    if (!plane.landn_state.diving) {
        // update crosstrack error, distance, and longitudinal distance
        plane.get_wp_crosstrack_error_m(plane.landn_state.xt_error);
        plane.get_wp_distance_m(plane.landn_state.wp_distance);
        plane.landn_state.longitudinal_wp_dist = safe_sqrt(MAX(sq(plane.landn_state.wp_distance)-sq(plane.landn_state.xt_error),0.0));
        // if crosstrack error
        if (abs(plane.landn_state.xt_error) > xt_acceptable || abs(ToDeg(plane.ahrs.roll)) > bank_acceptable) {
            // enter RTL
            plane.set_mode(plane.mode_rtl, ModeReason::APPROACH_UNHEALTHY);
            gcs().send_text(MAV_SEVERITY_ALERT, "LANDN: net approach unhealthy");
            return;
        }
        // we might have to navigate
        if (plane.landn_state.longitudinal_wp_dist <= landn_init_dist) {
            plane.landn_state.diving=true;
            plane.landn_state.locked_roll = true;
            plane.landn_state.locked_roll_err = 0;
            gcs().send_text(MAV_SEVERITY_ALERT, "LANDN: enter dive");
        } else {
            plane.calc_nav_roll();
            plane.calc_nav_pitch();
            // motors already off?
            if (!plane.landn_state.motors_off) {
                // time to cut motors off?
                if (plane.landn_state.longitudinal_wp_dist <= landn_init_dist + dt_motor_cut*plane.gps.ground_speed()) {
                    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
                    plane.landn_state.motors_off=true;
                    gcs().send_text(MAV_SEVERITY_ALERT, "LANDN: motors off");
                } else {
                    // otherwise let's calc throttle
                    plane.calc_throttle();
                }
            }
        }
    }
    
    // cut motor 0.2 s before and set plane.nav_pitch_cd = trim_pitch
    // do maneuver


    // handle locked/unlocked control
    if (plane.landn_state.locked_roll) {
        plane.nav_roll_cd = plane.landn_state.locked_roll_err;
    } else {
        plane.nav_roll_cd = plane.ahrs.roll_sensor;
    }
    if (plane.landn_state.locked_pitch) {
        plane.nav_pitch_cd = plane.landn_state.locked_pitch_cd;
    } else {
        plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
    }
}
