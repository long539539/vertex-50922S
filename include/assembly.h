#pragma once

#include "vex.h"

using namespace vex;

// The internals of this class is example code and can be deleted

// This is example code for a push back robot with two 5.5W motors on the lower intake,
// 11W motor on the top intake, a scraper, and wing

class Assembly {
public:
/* Create your devices here */
    static mik::motor_group cascade_motors;
    static mik::motor toggle_motor;
    static mik::piston pto_piston;
    static mik::motor claw_motor;
/* Examples of other vex devices you may need */
    static vex::rotation lift_rotation;
    static vex::optical optical_sensor;
    static vex::limit limit_switch;
    
    void init();
    void control();

    

    void lift_control();
    void lift_to_stage(int stage, int timeout_ms = 2000);
    void lift_to_deg(double target_deg, int timeout_ms = 2000); 
    void lift_auto_step();
    void score_macro(double drop_deg = 60, int timeout_ms = 800);
    void claw_control();
    void claw_spin(double volts, int ms);
    void toggle_motor_control();

    static bool color_stop_on;            // true = toggle_motor spins until it sees our color
    static bool sees_alliance_color();    // true when the sensor sees the color picked in the selector
    void spin_until_color(int timeout_ms = 1500);
    static vex::task optical_task;        // public so main.cpp can start it
    static int optical_loop();
   
private:
    

    
};