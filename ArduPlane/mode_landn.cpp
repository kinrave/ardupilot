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
    AP_GROUPINFO("XT_ACCPTBL", 1, ModeLandn, xt_acceptable, 2.5),

    // @Param: BNK_ACCPTB
    // @DisplayName: Acceptable bank
    // @Description: Acceptable absolute bank while in LANDN mode
    // @Range: 0 30
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("BNK_ACCPTB", 2, ModeLandn, bank_acceptable, 5),

    // @Param: DT_MOT_CUT
    // @DisplayName: Time befor Maneuver to cut motors
    // @Description: Time in seconds before initiating the LANDN dive to cut off the motor
    // @Units: s
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DT_MOT_CUT", 3, ModeLandn, dt_motor_cut, 0.1),

    // @Param: INIT_DIST
    // @DisplayName: Distance to init LANDN in zero wind
    // @Description: The distance to the last WP to init the LANDN dive.
    // @Units: m
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("INIT_DIST", 4, ModeLandn, landn_init_dist, 20),

    // @Param: RATE
    // @DisplayName: Pitch angle rate
    // @Description: Pitch angle rate to reach target angle
    // @Units: deg/s
    // @Range: -100 -10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE", 5, ModeLandn, landn_rate, -100),

    // @Param: TARGET_CD
    // @DisplayName: Target angle
    // @Description: Target angle for LANDN mode.
    // @Units: cdeg
    // @Range: -8900 -1000
    // @User: Standard
    AP_GROUPINFO("TARGET_CD", 6, ModeLandn, landn_target_cd, -7500),

    // @Param: WP_VDIST
    // @DisplayName: Vertical distance from net to last WP
    // @Description: The height the UAS must dive before reaching the net
    // @Units: m
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("WP_VDIST", 7, ModeLandn, wp_height_above_net, 10),

    // @Param: DT_MANEUV
    // @DisplayName: Time in decisec between impact and maneuver start
    // @Description: The time the UAV needs from initiating the maneuver until hitting the net (used to calculate maneuver wind correction)
    // @Units: decisec
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("DT_MANEUV", 8, ModeLandn, dt_maneuver_ds, 15),
    
    AP_GROUPEND
};

ModeLandn::ModeLandn()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeLandn::_enter()
{
    plane.landn_state.approach_WP = plane.prev_WP_loc;
    plane.landn_state.net_WP = plane.next_WP_loc;
    plane.landn_state.motors_off = false;
    plane.landn_state.diving = false;
    plane.landn_state.locked_roll = false;
    plane.landn_state.locked_pitch = false;
    plane.landn_state.landn_rate = constrain_float(landn_rate, -270, -10);
    plane.landn_state.landn_target_cd = constrain_int32(landn_target_cd, -8900, -1000);
    
    // update stage and log
    landn_stage = LANDN_ENTER;
    //landn_log();
    gcs().send_text(MAV_SEVERITY_INFO,"LANDN: entered");
    landn_stage = LANDN_NAVIGATE;

    return true;
}

void ModeLandn::update()
{
    // update crosstrack error, distance, and longitudinal distance
    plane.get_wp_crosstrack_error_m(plane.landn_state.xt_error);
    plane.get_wp_distance_m(plane.landn_state.wp_distance);
    plane.landn_state.longitudinal_wp_dist = safe_sqrt(MAX(sq(plane.landn_state.wp_distance)-sq(plane.landn_state.xt_error),0.0));
    Vector3f Plane2WP = plane.current_loc.get_distance_NED(plane.landn_state.net_WP);
    Plane2WP.z = 0;
    Vector3f vec_NEU_WPs = plane.landn_state.approach_WP.get_distance_NED(plane.landn_state.net_WP);
    vec_NEU_WPs.z = 0;
    // because Vectro3f.angle isnt defined above 90 deg
    if (Plane2WP * vec_NEU_WPs < 0) {
        plane.landn_state.longitudinal_wp_dist *= -1;
    }

    // check for diving
    if (!plane.landn_state.diving) {

        // update landn_init_dist
        float groundspeed_landn = plane.gps.ground_speed();
        float airspeed_landn;
        plane.ahrs.airspeed_estimate(airspeed_landn);
        float wind_speed_landn = airspeed_landn - groundspeed_landn;
        float landn_init_dist_corrected = landn_init_dist - wind_speed_landn * 0.1 * dt_maneuver_ds;

        // check approach health
        if (abs(plane.landn_state.xt_error) > xt_acceptable || abs(ToDeg(plane.ahrs.roll)) > bank_acceptable) {
            // enter RTL
            plane.set_mode(plane.mode_rtl, ModeReason::APPROACH_UNHEALTHY);
            // update stage and log
            landn_stage = ModeLandn::LANDN_STAGE::LANDN_RTL_EXIT;
            //landn_log();
            gcs().send_text(MAV_SEVERITY_WARNING, "LANDN: init_dist corr %.1f m",landn_init_dist_corrected);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "LANDN: bad approach %.1f m before",plane.landn_state.longitudinal_wp_dist);
            return;
        }

        // check for dive init
        if (plane.landn_state.longitudinal_wp_dist <= landn_init_dist_corrected) {
            plane.landn_state.diving=true;
            plane.landn_state.locked_roll = true;
            plane.landn_state.locked_roll_err = 0;
            // update stage and log
            landn_stage = LANDN_INIT_DIVE;
            //landn_log();
            gcs().send_text(MAV_SEVERITY_INFO, "LANDN: init dive %.1f m before",landn_init_dist_corrected);
            landn_stage = LANDN_PITCH_DOWN;
        } else { // else navigate
            plane.calc_nav_roll();
            plane.calc_nav_pitch();
            // motors already off?
            if (!plane.landn_state.motors_off) {
                // time to cut motors off?
                if (plane.landn_state.longitudinal_wp_dist <= landn_init_dist_corrected + dt_motor_cut*plane.gps.ground_speed()) {
                    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
                    plane.landn_state.motors_off=true;
                    // update stage and log
                    landn_stage = LANDN_CUT_MOTOR;
                    //landn_log();
                    gcs().send_text(MAV_SEVERITY_INFO, "LANDN: motors off");
                    landn_stage = LANDN_GLIDE;
                } else {
                    // otherwise let's calc throttle
                    plane.calc_throttle();
                }
            }
        }
    }

    // check net reached and send info
    int32_t dive_height_cm = (plane.landn_state.net_WP.alt-plane.current_loc.alt);
    if (landn_stage == LANDN_PITCH_DOWN && plane.landn_state.locked_pitch) {
        // update stage and log
        landn_stage = LANDN_LOCK_PITCH;
        //landn_log();
        gcs().send_text(MAV_SEVERITY_INFO,"LANDN: pitch locked");
        landn_stage = LANDN_DIVE;
    } else if (landn_stage == LANDN_DIVE && (abs(dive_height_cm) >= wp_height_above_net * 100)) {
        // update stage and log
        landn_stage = LANDN_NET_REACHED;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "LANDN: net reached at %.1f m",plane.landn_state.longitudinal_wp_dist);
        gcs().send_text(MAV_SEVERITY_INFO, "LANDN: xt error %.1f m",plane.landn_state.xt_error);
        gcs().send_text(MAV_SEVERITY_INFO, "LANDN: exit");
        //landn_log();
        landn_stage = LANDN_LANDN_END;
        AP_Notify::play_tune("MFT240L16O4MScP16<bP16b-bb-aa-P16gP16f#8gP16a-P16gP16f#gf#feP16e-P16d8e-");
    }
}

void ModeLandn::landn_log()
{
    float as_estimate;
    plane.ahrs.airspeed_estimate(as_estimate);
    Vector3f gsVec;
    plane.ahrs.get_velocity_NED(gsVec);
    Vector3f wVec = plane.ahrs.wind_estimate();
    float dive_height = (plane.landn_state.net_WP.alt-plane.current_loc.alt) * 0.01;
    plane.get_wp_crosstrack_error_m(plane.landn_state.xt_error);
    plane.get_wp_distance_m(plane.landn_state.wp_distance);
    plane.landn_state.longitudinal_wp_dist = safe_sqrt(MAX(sq(plane.landn_state.wp_distance)-sq(plane.landn_state.xt_error),0.0));
    AP::logger().Write("LDN1", "TimeUS,state,R,P,Y,AS,gsX,gsY,gsZ", "QBfffffff",
                                        AP_HAL::micros64(),
                                        landn_stage,
                                        (double)ToDeg(plane.ahrs.roll),
                                        (double)ToDeg(plane.ahrs.pitch),
                                        (double)ToDeg(plane.ahrs.yaw),
                                        (double)as_estimate,
                                        (double)gsVec.x,
                                        (double)gsVec.y,
                                        (double)gsVec.z);
    AP::logger().Write("LDN2", "TimeUS,state,wX,wY,wZ,alt,div_hgt,dist,xt,dist_along,elev,thr", "QBffffffffff",
                                        AP_HAL::micros64(),
                                        landn_stage,
                                        (double)wVec.x,
                                        (double)wVec.y,
                                        (double)wVec.z,
                                        (double)plane.adjusted_relative_altitude_cm()*0.01,
                                        (double)dive_height,
                                        (double)plane.landn_state.wp_distance,
                                        (double)plane.landn_state.xt_error,
                                        (double)plane.landn_state.longitudinal_wp_dist,
                                        (double)SRV_Channels::get_output_norm(SRV_Channel::k_elevator),
                                        (double)SRV_Channels::get_output_norm(SRV_Channel::k_throttle));
    if (landn_stage == LANDN_NET_REACHED) {
        // prepare dLon, dLat, dive, inertial speed, inertial direction (with respect to wp-dir) and pitch
        Vector3f Plane2WP = plane.current_loc.get_distance_NED(plane.landn_state.net_WP);
        Plane2WP.z = 0;
        Vector3f vec_NEU_WPs = plane.landn_state.approach_WP.get_distance_NED(plane.landn_state.net_WP);
        vec_NEU_WPs.z = 0;
        // because Vectro3f.angle isnt defined above 90 deg
        if (Plane2WP * vec_NEU_WPs < 0) {
            plane.landn_state.longitudinal_wp_dist *= -1;
        }
        // check -90 deg overshoot
        float pitch = ToDeg(plane.ahrs.pitch);
        if (abs(ToDeg(plane.ahrs.roll))>90) {
            pitch = -180 - pitch;
        }
        // Vector3f.angle does only calculate angles < 90 deg
        float impact_angle = ToDeg(gsVec.angle(vec_NEU_WPs));
        if (gsVec * vec_NEU_WPs < 0) {
            impact_angle = 180 - impact_angle;
        }
        // maximum 50 characters
        gcs().send_text(MAV_SEVERITY_INFO, "dx= %.1f,dy= %.1f,v= %.0f,ang= %.0f,P= %.0f,p= %.0f",
                                            plane.landn_state.longitudinal_wp_dist,
                                            plane.landn_state.xt_error,
                                            gsVec.length(),
                                            impact_angle,
                                            pitch,
                                            ToDeg(plane.ahrs.get_gyro().y));
    }
}