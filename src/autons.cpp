#include "vex.h"
#include "rcl-config.h"

// Watches the claw distance sensor during a non-blocking drive and toggles the claw
// as soon as something is in range. Returns false if the motion ended without seeing anything



void default_constants() {
    // Each controller constant in the form of throttle, turn (deadband, min_output, curve_gain), desaturate_bias.
    chassis.set_control_constants(5, 10, 1.019, 5, 10, 1.019, 0.5);

    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI, slew).
    chassis.set_drive_constants(10, 1.4, 0.025, 10.3, 3, 0);
    chassis.set_heading_constants(10, .31, 0, 2.4, 0, 0);
    chassis.set_turn_constants(12, .4, .03, 3, 15, 0);
    chassis.set_swing_constants(12, .4, .01, 2.95, 15, 0);

    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_turn_exit_conditions(1, 200, 3000);
    chassis.set_drive_exit_conditions(2, 200, 5000);
    chassis.set_swing_exit_conditions(1, 200, 3000);

    constants.boomerang_lead = .5;
    constants.boomerang_drift = 2;
}

/* When creating autons with odometry, use a path planner, https://path.jerryio.com/.
If your robot is not lining up with the points, double check you set up your odometry correctly */

std::string blue_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue left winpoint";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(-62.5, -7, 90);

        return "";
    }
    RclMain.startTracking(); 
    
    vex::task rcl_debug([]() {
        auto dist = [](vex::distance& d) {
            if (!d.installed()) return std::string("unplugged");
            if (!d.isObjectDetected()) return std::string("none");
            return to_string_float(d.objectDistance(vex::inches), 1) + " in";
        };
        while (true) {
            RclPose p = RclMain.getRclPose();
            print("odom (" + to_string_float(chassis.get_X_position(), 2) + ", " + to_string_float(chassis.get_Y_position(), 2) +
                  ")   rcl (" + to_string_float(p.x, 2) + ", " + to_string_float(p.y, 2) + ")" +
                  "   left " + dist(left_dist) + "   right " + dist(right_dist) + "   rear " + dist(rear_dist));
            vex::task::sleep(250);
        }
        return 0;
    });



    // //spin tthe toggle to alliance color
    assembly.spin_until_color(750);
    


    // //score first pin
    // chassis.drive_to_pose(-50, -22, 180, {.max_voltage = 12, .wait = false, .timeout = 2000});
    // assembly.lift_to_stage(1); 
    // chassis.wait();
    // assembly.score_macro(100);
    // chassis.drive_to_pose(-47, -7, 180, {.max_voltage = 12, .wait = false});
    // assembly.lift_to_stage(0);
    // chassis.wait();

    // //Pick up the pin/cup
    // chassis.drive_to_pose(-30, -23, 135, {.min_voltage = 4, .max_voltage = 12, .exit_error = 4, .timeout = 1500});
    // chassis.stop_drive(brakeType::hold);
    // assembly.claw_piston.toggle();
    // wait(300, msec);

    // //Score the pin/cup and run to a safe position to turn
    // chassis.drive_to_pose(-54, -22, 270, {.max_voltage = 12, .wait = false, .timeout = 1500, .direction = fwd});
    // assembly.lift_to_stage(3); 
    // chassis.wait();
    // assembly.score_macro(100);
    // chassis.drive_to_pose(-30, -22, 270, {.max_voltage = 12, .wait = false});
    // assembly.lift_to_stage(0);
    // chassis.wait();

    // //Pick up the pin/cup
    // chassis.drive_to_pose(-60, -54, 225, {.min_voltage = 4, .max_voltage = 12, .exit_error = 4, .timeout = 1500});
    // chassis.stop_drive(brakeType::hold);
    // assembly.claw_piston.toggle();
    // wait(300, msec);

    // //Score the pin/cup 
    // chassis.drive_to_pose(-50, -20, 0, {.max_voltage = 12, .wait = false, .timeout = 1500, .direction = fwd});
    // assembly.lift_to_stage(5); 
    // chassis.wait();
    // assembly.score_macro(100);

    // //Drive and turn to a position so that we can easily load pin/cup
    // chassis.drive_to_pose(-51, -57, 0, {.max_voltage = 12, .wait = false, .timeout = 1500});
    // assembly.lift_to_stage(0);
    // chassis.wait();
	// chassis.turn_to_angle(270);
    
    return "";
}
std::string blue_left_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue left sawp";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string blue_left_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue left elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string blue_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue right winpoint";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(-7, -62.5, 0);

        return "";
    }
    // Place start of autonoumous here
    
    
    return "";
}
std::string blue_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue right sawp";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string blue_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue right elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}

std::string red_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red left winpoint";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(-62.5, -7, 90);

        return "";
    }

    // Place start of autonoumous here


    return "";
}
std::string red_left_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red left sawp";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_left_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red left elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red right winpoint";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(-7, -62.5, 0);

        return "";
    }
    // Place start of autonoumous here

    return "";
}
std::string red_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red right sawp";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red right elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string skills(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "skills";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        chassis.set_coordinates(-62, 0, 90);
    
        return "";
    }
   
    return "";
}
