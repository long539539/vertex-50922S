#include "vex.h"

// The code in this file is example and can be deleted

// This is example code for a push back robot with two 5.5W motors on the lower intake,
// 11W motor on the top intake, a scraper, and wing
static constexpr double claw_grab_distance_mm = 30.0;
static constexpr int claw_macro_poll_ms = 20;
static constexpr int lift_stage_count = 6;              // stage 0 + 5 stages
static constexpr int lift_top_stage = lift_stage_count - 1;

// Lift height as degrees on lift_rotation, counted up from where the lift was
// when the program started (see pre_auton in main.cpp). These numbers are
// placeholders: drive the lift to each stage and copy the value off the
// controller readout
static constexpr double lift_stage_deg[lift_stage_count] = {
    0,        // stage 0 - bottom / rest, where the lift sits when the program starts
    400,   // stage 1
    800.0,   // stage 2
    1200,   // stage 3
    1600.0,   // stage 4
    3000.0,   // stage 5 - top, L1 will not drive past this
};

// A stage counts as reached as soon as the sensor hits its angle, but it only
// counts as left once the lift drops this far back below it. Without the gap,
// a parked lift wobbling a few degrees in hold flips the stage back and forth and rumbles.
static constexpr double lift_stage_hysteresis_deg = 10.0;

static constexpr double lift_up_volt = 12.0;
static constexpr double lift_down_volt = -12.0;  // gravity does half the work going down
static constexpr int lift_macro_poll_ms = 20;

// Auton lift PID for lift_to_stage(), same idea as the chassis constants in
// default_constants(). Tune on the robot:
//   flies past the stage  -> raise lift_kd
//   slow getting there    -> raise lift_kp
//   stops short going up  -> raise lift_ki
static constexpr double lift_kp = 0.05;           // volts per degree away, full 12 V from 240 deg out
static constexpr double lift_ki = 0.003;          // pushes through gravity in the last few degrees
static constexpr double lift_kd = 0.5;            // brakes the lift as it closes in on the stage
static constexpr double lift_starti_deg = 40.0;   // ki only builds up this close to the stage
static constexpr double lift_settle_error_deg = 15.0;
static constexpr double lift_settle_time_ms = 100.0;  // must stay inside settle error this long to finish

// lift_to_stage() gives up once the lift is this much further from the stage
// than where it started, because that means it is moving the wrong way
static constexpr double lift_wrong_way_deg = 50.0;

// Re-sending stop(hold) every poll makes the motors re-grab their hold position
// and sag, so the loop only sends a command when the command actually changes
enum lift_cmd { LIFT_STOP, LIFT_UP, LIFT_DOWN };
// This function is called once at the start of user control
void Assembly::init() {
    // You can declare a task that you want to always run in user control.
    claw_macro_task = vex::task(claw_macro_loop);

    // The lift owns L1/L2 from its own task, see lift_macro_loop()
    lift_macro_task = vex::task(lift_macro_loop);
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

    two_bar_control();
    claw_piston_control();
}


// Spins intake forward if L1 is being held, reverse if L2 is being held; stops otherwise
void Assembly::two_bar_control() {
    if (Controller.ButtonR2.pressing()) {
        two_bar_motor.spin(fwd, 12, volt);
    } else if (Controller.ButtonR1.pressing()) {
        two_bar_motor.spin(fwd, -12, volt);
    } else {
        two_bar_motor.stop();
    }
}


// Extends piston when button R1 is pressed, releases otherwise 
void Assembly::claw_piston_control() {
    if (btnA_new_press(Controller.ButtonA.pressing())) {
        claw_piston.toggle();
    }
}

int Assembly::claw_macro_loop() {
    int readout_timer = 0;

    while (true) {
        double dist = distanceclaw.objectDistance(mm);

        // The sensor reports a very large value when it sees nothing, so bound
        // both ends instead of trusting a bare "less than" check
        bool in_range = dist > 0 && dist <= claw_grab_distance_mm;

        if (Controller.ButtonDown.pressing()) {
            if (in_range && !claw_macro_grabbed) {
                claw_macro_grabbed = true;
                claw_piston.toggle();
                Controller.rumble(".");
            }
        } else {
            // Releasing Down re-arms the macro for the next grab
            claw_macro_grabbed = false;
        }

        // Live readout so you can tune claw_grab_distance_mm against the real claw
        readout_timer += claw_macro_poll_ms;
        if (readout_timer >= 250) {
            readout_timer = 0;
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Claw:%6.0fmm %s", dist,
                                    claw_macro_grabbed ? "GRAB" : (in_range ? "IN  " : "OUT "));
        }

        task::sleep(claw_macro_poll_ms);
    }
    return 0;
}

// 5 stage lift.
//
// Hold L1 and the lift rises; every time it crosses one of the angles in
// lift_stage_deg the controller rumbles once, so you can count the stages going
// by without looking. It stops itself at stage 5 instead of driving into the
// hard stop. Hold L2 and it comes back down the same way, rumbling on each
// stage. Let go of both buttons and the lift holds where it is.
//
// There is no bottom switch, so the sensor's zero is wherever the lift was
// when the program started. Always start the program with the lift all the
// way down.
int Assembly::lift_macro_loop() {
    int readout_timer = 0;
    int last_cmd = -1;     // -1 so the first pass always sends a command
    bool seeded = false;   // the first real reading sets the stage, it does not "pass" one

    cascade_motors.setStopping(brakeType::hold);

    while (true) {
        double height = lift_rotation.position(deg);

        // The rotation sensor always has a reading unless it is unplugged, so
        // that is the only case where the reading gets thrown out
        bool height_valid = lift_rotation.installed();

        /* ---- which stage are we on ---- */
        int stage = lift_stage;

        if (height_valid) {
            // Climb up through every stage we have reached...
            while (stage < lift_top_stage && height >= lift_stage_deg[stage + 1]) {
                stage++;
            }
            // ...and back down through every stage we have dropped clear of
            while (stage > 0 && height < lift_stage_deg[stage] - lift_stage_hysteresis_deg) {
                stage--;
            }
        }
        // Sensor unplugged: keep the last known stage rather than guessing a new one

        if (stage != lift_stage) {
            // Booting with the lift already raised would otherwise rumble once
            // per stage while it works out where it is
            if (seeded) Controller.rumble(".");  // one buzz per stage passed, up or down
            lift_stage = stage;
        }
        if (height_valid) seeded = true;

        /* ---- drive the lift ---- */
        int cmd = LIFT_STOP;
        const char* state = "HLD";

        if (control_disabled()) {
            // Auton selector / field disable. The task keeps polling, the lift does not move
            state = "OFF";
        } else if (Controller.ButtonL1.pressing()) {
            if (lift_stage >= lift_top_stage) {
                state = "TOP";  // top stage, hold instead of pushing into the hard stop
            } else {
                cmd = LIFT_UP;
                state = "UP ";
            }
        } else if (Controller.ButtonL2.pressing()) {
            // No bottom stop: if the zero is ever off, a soft stop here could
            // leave the lift stuck up. The driver lets go at the bottom
            cmd = LIFT_DOWN;
            state = "DWN";
        }

        if (cmd != last_cmd) {
            if (cmd == LIFT_UP) {
                cascade_motors.spin(fwd, lift_up_volt, volt);
            } else if (cmd == LIFT_DOWN) {
                cascade_motors.spin(fwd, lift_down_volt, volt);
            } else {
                cascade_motors.stop(brakeType::hold);
            }
            last_cmd = cmd;
        }

        /* ---- live readout, this is what you measure lift_stage_deg with ---- */
        readout_timer += lift_macro_poll_ms;
        if (readout_timer >= 250) {
            readout_timer = 0;
            Controller.Screen.setCursor(2, 1);
            // Printed as an int, not %f: float printf only works on the V5 when
            // the project is linked with the print-float flag, and whole
            // degrees are plenty for picking stage angles
            if (height_valid) {
                Controller.Screen.print("Lift:%4ddeg S%d %s", (int)height, lift_stage, state);
            } else {
                Controller.Screen.print("Lift: ---deg S%d %s", lift_stage, state);
            }
        }

        task::sleep(lift_macro_poll_ms);
    }
    return 0;
}

// Auton only: drives the lift to a stage and waits until it gets there.
// Uses the same PID class as the chassis: the D term brakes the lift on the
// way in instead of letting it fly past, and it only finishes once the lift
// has sat at the stage for lift_settle_time_ms. Gives up after timeout_ms so
// a jammed lift cannot freeze the auton
void Assembly::lift_to_stage(int stage, int timeout_ms) {
    if (stage < 0) stage = 0;
    if (stage > lift_top_stage) stage = lift_top_stage;
    double target = lift_stage_deg[stage];

    PID lift_pid(lift_kp, lift_ki, lift_kd, lift_starti_deg,
                 lift_settle_error_deg, lift_settle_time_ms, 0, timeout_ms);
    double start_error = std::abs(target - lift_rotation.position(deg));

    while (!lift_pid.is_settled()) {
        double error = target - lift_rotation.position(deg);

        // Further from the stage than when we started: the lift is going the
        // wrong way (sensor reversed, or not reading the lift). Stop and
        // rumble instead of driving into the hard stop until the timeout
        if (std::abs(error) > start_error + lift_wrong_way_deg) {
            Controller.rumble("---");
            break;
        }

        double volts = lift_pid.compute(error);
        if (volts > lift_up_volt) volts = lift_up_volt;
        if (volts < lift_down_volt) volts = lift_down_volt;

        cascade_motors.spin(fwd, volts, volt);
        task::sleep(10);  // mikLib's PID counts time in 10 ms steps
    }
    cascade_motors.stop(brakeType::hold);
}