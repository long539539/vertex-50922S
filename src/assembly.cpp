#include "vex.h"

// The code in this file is example and can be deleted

// This is example code for a push back robot with two 5.5W motors on the lower intake,
// 11W motor on the top intake, a scraper, and wing


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

    if (Controller.ButtonL1.pressing() || Controller.ButtonL2.pressing()) {
        color_stop_on = false;   // the optical task stops driving the motor while the lift uses it
        lift_control();
    } else {
        toggle_motor_control();
    }
}


// Spins intake forward if L1 is being held, reverse if L2 is being held; stops otherwise


// Lift height for each stage, in rotation sensor degrees. Stage 1 = bottom = 0
static constexpr int lift_stage_count = 9;
static constexpr double lift_stage_deg[lift_stage_count] = {30, 200, 280, 350, 460, 600, 650, 750, 2250};

static constexpr double lift_kp = 0.25;            // volts per degree of error
static constexpr double lift_max_volt = 12;       // going up
static constexpr double lift_down_max_volt = 9;   // going down
static constexpr double lift_tolerance_deg = 20;  // this close = at the stage

// Driver: raises the lift while L1 is held, lowers it while L2 is held; holds its height otherwise
void Assembly::lift_control() {
    double height = lift_rotation.position(deg);   // read the sensor once per loop
    if (Controller.ButtonL1.pressing()) {
        pto_piston.close();
        cascade_motors.spin(fwd, 12, volt);
        toggle_motor.spin(fwd, 12, volt);
    } else if (Controller.ButtonL2.pressing() && height > lift_stage_deg[0]+30) {
        pto_piston.close();
        cascade_motors.spin(fwd, -12, volt);
        toggle_motor.spin(fwd, -12, volt);
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
    if (stage > lift_stage_count) stage = lift_stage_count;
    double target = lift_stage_deg[stage];
    pto_piston.close();
    for (int t = 0; t < timeout_ms; t += 10) {
        double error = target - lift_rotation.position(deg);
        if (std::abs(error) < lift_tolerance_deg) break;

        double volts = clamp(error * lift_kp, -lift_down_max_volt, lift_max_volt);
        cascade_motors.spin(fwd, volts, volt);
        toggle_motor.spin(fwd, volts, volt);
        task::sleep(10);
    }
    cascade_motors.stop(brakeType::hold);
    toggle_motor.stop(brakeType::hold);
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
    // Light on, so colors read the same in any room lighting
    optical_sensor.setLight(ledState::on);
    optical_sensor.setLightPower(100);
    optical_sensor.integrationTime(5); 

    while (true) {
        if (color_stop_on) {
            pto_piston.open();
            if (sees_alliance_color()) {
                toggle_motor.stop(brakeType::hold);
            } else {
                toggle_motor.spin(fwd, 12, volt);
            }
        }

        // Every task loop needs a sleep, or it hogs the brain and the drive stops responding
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
        color_stop_on = true;    // the optical task controls the motor
        return;                  // don't touch the motor here, or the two fight
    }
    color_stop_on = false;

    if (Controller.ButtonB.pressing()) {
        pto_piston.open(); 
        toggle_motor.spin(fwd, -12, volt);
    } else {
        toggle_motor.stop(brakeType::hold);
    }
}


