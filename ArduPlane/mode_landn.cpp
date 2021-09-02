#include "mode.h"
#include "Plane.h"

bool ModeLandn::_enter()
{
    plane.landn_state.wait_for_maneuver = true;
    plane.landn_state.locked_roll = false;
    plane.landn_state.locked_pitch = false;
    plane.landn_state.initial_pitch = plane.ahrs.pitch_sensor;

    return true;
}

void ModeLandn::update()
{
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
