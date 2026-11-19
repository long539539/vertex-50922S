#include "vex.h"
static bool lift_auto_on = false;                    // true while the Up macro is running
static constexpr double lift_auto_target_deg = 190; //190;  // where the Up macro sends the lift
static double pto_closed_at = -100000;   // when the PTO last closed; big negative = no wait at startup

// The code in this file is example and can be deleted




// This function is called once at the start of user control
void Assembly::init() {
    // You can declare a task that you want to always run in user control.


    // For example a task that is always checking if the intake is moving forward while being told to,
    // and detected not, then the intake will move in reverse to unjam itself
}


// You want to put this function inside the user control loop in main.
void Assembly::control() {
    static bool initialized = false;

    if (!initialized) {
        init();
        initialized = true;
    }

    claw_control();

    if (Controller.ButtonUp.pressing()) {
        lift_auto_on = true;      // start the macro
        color_stop_on = false;    // keep the optical task from opening the PTO mid-lift
    }

    if (Controller.ButtonL1.pressing() || Controller.ButtonL2.pressing()) {
        color_stop_on = false;
        lift_control();
    } else if (lift_auto_on) {
        lift_auto_step();  
    } else {
        cascade_motors.stop(brakeType::hold); 
        toggle_motor_control();
    }
    
}


// Spins intake forward if L1 is being held, reverse if L2 is being held; stops otherwise


// Lift height for each stage, in rotation sensor degrees. Stage 1 = bottom = 0
static constexpr int lift_stage_count = 9;
static constexpr double lift_stage_deg[lift_stage_count] = {30, 195, 280, 350, 460, 555, 650, 750, 1000};

static constexpr double lift_kp = 0.25;            // volts per degree of error
static constexpr double lift_max_volt = 12;       // going up
static constexpr double lift_down_max_volt = 9;   // going down
static constexpr double lift_tolerance_deg = 20;  // this close = at the stage

// Driver: raises the lift while L1 is held, lowers it while L2 is held; holds its height otherwise
void Assembly::lift_control() {
    double height = lift_rotation.position(deg);       // read the sensor once per loop
    bool up   = Controller.ButtonL1.pressing();
    bool down = Controller.ButtonL2.pressing() && height > lift_stage_deg[0]+30;

    if (up || down) {
        if (pto_piston.state()) {                       // PTO was open: close it and start the 100ms wait
            pto_piston.close();
            pto_closed_at = Brain.Timer.time(msec);
        }

        double volts = up ? 12 : -12;
        cascade_motors.spin(fwd, volts, volt);          // cascade starts right away

        if (Brain.Timer.time(msec) - pto_closed_at >= 100) {
            toggle_motor.spin(fwd, volts, volt);        // PTO has had 100ms to engage
        } else {
            toggle_motor.stop(brakeType::coast);        // free-spin so the PTO gears can mesh
        }
    } else {
        cascade_motors.stop(brakeType::hold);
        toggle_motor.stop(brakeType::hold);
    }
}


// Auton: moves the lift to a stage (1 = bottom, 8 = top) and waits until it gets there.
// Every loop: error = target - sensor, motor volts = error * kp.
// Gives up after timeout_ms so a jam can't freeze the auton
void Assembly::lift_to_stage(int stage, int timeout_ms) {
    if (stage < 0) stage = 0;
    if (stage > lift_stage_count - 1) stage = lift_stage_count - 1;   // see note below
    double target = lift_stage_deg[stage];
    pto_piston.close();
    for (int t = 0; t < timeout_ms; t += 10) {
        double error = target - lift_rotation.position(deg);
        if (std::abs(error) < lift_tolerance_deg) break;

        double volts = clamp(error * lift_kp, -lift_down_max_volt, lift_max_volt);
        cascade_motors.spin(fwd, volts, volt);       // cascade starts right away

        if (t >= 100) {
            toggle_motor.spin(fwd, volts, volt);     // PTO has had 100ms to engage
        } else {
            toggle_motor.stop(brakeType::coast);     // free-spin so the PTO gears can mesh
        }
        task::sleep(10);
    }
    cascade_motors.stop(brakeType::hold);
    toggle_motor.stop(brakeType::hold);
}

void Assembly::lift_to_deg(double target_deg, int timeout_ms) {
    target_deg = clamp(target_deg, lift_stage_deg[0], lift_stage_deg[lift_stage_count - 1]);

    pto_piston.close();
    for (int t = 0; t < timeout_ms; t += 10) {
        double error = target_deg - lift_rotation.position(deg);
        if (std::abs(error) < lift_tolerance_deg) break;

        double volts = clamp(error * lift_kp, -lift_down_max_volt, lift_max_volt);
        cascade_motors.spin(fwd, volts, volt);       // cascade starts right away

        if (t >= 100) {
            toggle_motor.spin(fwd, volts, volt);     // PTO has had 100ms to engage
        } else {
            toggle_motor.stop(brakeType::coast);     // free-spin so the PTO gears can mesh
        }
        task::sleep(10);
    }
    cascade_motors.stop(brakeType::hold);
    toggle_motor.stop(brakeType::hold);
}

// Driver macro: moves the lift ONE step toward lift_auto_target_deg.
// Called every driver loop while lift_auto_on is true, so you can still drive.
void Assembly::lift_auto_step() {
    double error = lift_auto_target_deg - lift_rotation.position(deg);

    if (std::abs(error) < lift_tolerance_deg) {   // close enough: macro done
        lift_auto_on = false;
        cascade_motors.stop(brakeType::hold);
        toggle_motor.stop(brakeType::hold);
        return;
    }

    if (pto_piston.state()) {                      // PTO was open: close it and start the 100ms wait
        pto_piston.close();
        pto_closed_at = Brain.Timer.time(msec);
    }

    double volts = clamp(error * lift_kp, -lift_down_max_volt, lift_max_volt);
    cascade_motors.spin(fwd, volts, volt);         // cascade starts right away

    if (Brain.Timer.time(msec) - pto_closed_at >= 100) {
        toggle_motor.spin(fwd, volts, volt);       // PTO has had 100ms to engage
    } else {
        toggle_motor.stop(brakeType::coast);       // free-spin so the PTO gears can mesh
    }
}


// Auton: lowers the lift drop_deg from where it is now, then toggles the claw to let go.
// The waits let the lift settle and the piston fire before the robot moves again
void Assembly::score_macro(double drop_deg, int timeout_ms) {
    double target = lift_rotation.position(deg) - drop_deg;
    if (target < lift_stage_deg[0]) target = lift_stage_deg[0];   // never drive below the bottom stage
    for (int t = 0; t < timeout_ms; t += 10) {
        double error = target - lift_rotation.position(deg);
        if (std::abs(error) < lift_tolerance_deg) break;

        double volts = clamp(error * lift_kp, -lift_down_max_volt, lift_max_volt);
        cascade_motors.spin(fwd, volts, volt);
        toggle_motor.spin(fwd, volts, volt);        task::sleep(10);
    }
    cascade_motors.stop(brakeType::hold);
    toggle_motor.stop(brakeType::hold);
    claw_spin(-12, 200);  

}




// Extends piston when button R1 is pressed, releases otherwise
// Ratchet claw: hold R1 to intake, hold R2 to outtake
void Assembly::claw_control() {
    if (Controller.ButtonR1.pressing()) {
        claw_motor.spin(fwd, 12, volt);
    } else if (Controller.ButtonR2.pressing()) {
        claw_motor.spin(fwd, -12, volt);
    } else {
        claw_motor.stop(brakeType::hold);
    }
}

// Auton: spins the claw for ms milliseconds, then holds. 12 = intake, -12 = outtake
void Assembly::claw_spin(double volts, int ms) {
    claw_motor.spin(fwd, volts, volt);
    wait(ms, msec);
    claw_motor.stop(brakeType::hold);
}



// True when the optical sensor sees the alliance color picked in the auton selector
bool Assembly::sees_alliance_color() {
    if (!optical_sensor.isNearObject()) return false;   // nothing in front, ignore the hue
    double hue = optical_sensor.hue();

    if (auton_scr->get_alliance_color() == mik::alliance_colors::RED) {
        return hue >= 350 || hue <= 10;    // red
    }
    return hue >= 190 && hue <= 250;    // blue
    // return hue >= 350 || hue <= 10;     //red
}

// Runs the whole match (started in pre_auton). While color_stop_on is true,
// spins toggle_motor and stops it when our alliance color is seen
int Assembly::optical_loop() {
    optical_sensor.setLight(ledState::on);
    optical_sensor.setLightPower(100);
    optical_sensor.integrationTime(5); 

    while (true) {
        if (color_stop_on) {
            pto_piston.open();
            if (sees_alliance_color()) {
                toggle_motor.stop(brakeType::hold);
            } else {
                toggle_motor.spin(fwd, -12, volt);
            }
        }

        task::sleep(10);
    }
    return 0;
}

// Auton: spins toggle_motor until our color is seen, then stops. Gives up after timeout_ms
void Assembly::spin_until_color(int timeout_ms) {
    pto_piston.close();
    for (int t = 0; t < timeout_ms; t += 10) {
        if (sees_alliance_color()) break;
        toggle_motor.spin(fwd, 12, volt);
        task::sleep(10);
    }
    toggle_motor.stop(brakeType::hold);
}

// Hold Down = color stop mode. Otherwise each press of B flips toggle_motor on/off
void Assembly::toggle_motor_control() {
    if (Controller.ButtonDown.pressing()) {
        pto_piston.open(); 
        wait(100, msec);              // let the piston open before spinning
        color_stop_on = true;    // the optical task controls the motor
        return;                  // don't touch the motor here, or the two fight
    }
    color_stop_on = false;

    if (Controller.ButtonB.pressing()) {
        pto_piston.open(); 
        wait(100, msec);              // let the piston open before spinning
        toggle_motor.spin(fwd, -12, volt);
    } else {
        toggle_motor.stop(brakeType::hold);
    }
}



// // ============================ ANTI-TIP ============================
// // Sign used everywhere below:  tilt + = nose dipping (tipping forward)
// //                              tilt - = back dipping (tipping backward)

// static constexpr bool   tip_enabled       = true;   // false = anti-tip off (setup / kill switch)

// // --- Set these from the setup test (anti_tip_debug) ---
// static constexpr bool   tip_use_pitch     = true;   // false = your front/back tilt shows up on roll
// static constexpr double tip_sign          = 1.0;    // -1.0 if nose-down makes the number go negative
// static constexpr double tip_level_offset  = 0.0;    // raw reading with the robot sitting flat

// // --- Anti-tip tuning ---
// static constexpr double tip_trigger_deg   = 7.0;    // start correcting above this tilt
// static constexpr double tip_release_deg   = 2.5;    // stop correcting below this tilt
// static constexpr double tip_give_up_deg   = 40.0;   // past this the robot has fallen, stop trying
// static constexpr int    tip_confirm_loops = 3;      // readings in a row over trigger (3 x 5 ms)
// static constexpr double tip_kp            = 0.8;    // volts per degree of tilt
// static constexpr double tip_kd            = 0.0;    // volts per deg/sec, leave 0 until kp is tuned
// static constexpr double tip_min_volt      = 3.0;    // smallest push that actually moves the robot
// static constexpr double tip_max_volt      = 12.0;
// static constexpr double tip_lift_min_deg  = 0;      // only correct when lift is above this (0 = always)
// static constexpr double tip_handback_ms   = 300;    // sticks fade back in over this long after a save

// static bool   tip_active       = false;
// static int    tip_count        = 0;
// static double tip_last         = 0;
// static double tip_rate         = 0;          // deg/sec, smoothed
// static double tip_last_time    = 0;
// static double tip_release_time = -100000;    // big negative = no fade at startup

// // Front/back tilt in degrees, + = nose down
// double Assembly::get_tilt() {
//     double raw = tip_use_pitch ? chassis.inertial.pitch() : chassis.inertial.roll();
//     return reduce_negative_180_to_180(raw - tip_level_offset) * tip_sign;
// }

// // Call every driver loop BEFORE chassis.control(). Returns true when it is driving
// // the robot (saving it, or fading the sticks back in), so skip split arcade that loop.
// bool Assembly::anti_tip_step() {
//     if (!tip_enabled || !chassis.inertial.installed() || chassis.calibrating) {
//         tip_active = false;
//         return false;
//     }

//     double now  = Brain.Timer.time(msec);
//     double tilt = get_tilt();

//     // How fast the tilt is changing (deg/sec). Smoothed because the IMU
//     // updates a little slower than this 5 ms loop
//     double dt = (now - tip_last_time) / 1000.0;
//     if (dt > 0) {
//         tip_rate = 0.7 * tip_rate + 0.3 * ((tilt - tip_last) / dt);
//     }
//     tip_last      = tilt;
//     tip_last_time = now;

//     // Already fallen over: spinning the wheels won't help
//     if (std::abs(tilt) > tip_give_up_deg) {
//         tip_active = false;
//         tip_count  = 0;
//         return false;
//     }

//     if (!tip_active) {
//         // Several readings in a row over the trigger, so one noisy reading can't set it off
//         bool lift_high = lift_rotation.position(deg) >= tip_lift_min_deg;
//         if (lift_high && std::abs(tilt) > tip_trigger_deg) tip_count++;
//         else tip_count = 0;

//         if (tip_count >= tip_confirm_loops) {
//             tip_active = true;
//             tip_count  = 0;
//             Controller.rumble(".");
//         }
//     } else if (std::abs(tilt) < tip_release_deg) {
//         tip_active       = false;
//         tip_release_time = now;     // start fading the sticks back in
//     }

//     // Saving the robot: drive TOWARD the tip (nose down = forward, back down = backward)
//     if (tip_active) {
//         double push = tip_kp * tilt;
//         if (std::abs(push) < tip_min_volt) push = (tilt > 0) ? tip_min_volt : -tip_min_volt;
//         double volts = clamp(push + tip_kd * tip_rate, -tip_max_volt, tip_max_volt);
//         chassis.drive_with_voltage(volts, volts);   // same on both sides = no turning
//         return true;
//     }

//     // Just finished a save: split arcade, but faded 0 -> 100% over tip_handback_ms
//     // so the driver's stick doesn't instantly tip the robot again
//     double since_release = now - tip_release_time;
//     if (since_release < tip_handback_ms) {
//         double fade     = since_release / tip_handback_ms;
//         double throttle = deadband(Controller.Axis3.value(), constants.control_throttle_deadband) * fade;
//         double turn     = deadband(Controller.Axis1.value(), constants.control_turn_deadband) * 0.6 * fade;
//         chassis.drive_with_voltage(percent_to_volt(throttle + turn), percent_to_volt(throttle - turn));
//         return true;
//     }

//     return false;   // normal driving: chassis.control(SPLIT_ARCADE) takes over
// }

// // Setup only: shows raw pitch/roll and the computed tilt on the controller
// void Assembly::anti_tip_debug() {
//     static double last_print = 0;
//     double now = Brain.Timer.time(msec);
//     if (now - last_print < 200) return;   // controller screen is slow, don't spam it
//     last_print = now;

//     Controller.Screen.setCursor(1, 1);
//     Controller.Screen.print("P:%6.1f R:%6.1f  ", chassis.inertial.pitch(), chassis.inertial.roll());
//     Controller.Screen.setCursor(2, 1);
//     Controller.Screen.print("tilt:%5.1f %s  ", get_tilt(), tip_active ? "SAVE" : "off ");
// }
