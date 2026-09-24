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
    static mik::motor two_bar_motor;
    static mik::piston claw_piston;
    static vex::distance distanceclaw;
    static vex::rotation lift_rotation;
    static vex::bumper lift_bottom_left;
    static vex::bumper lift_bottom_right;
/* Examples of other vex devices you may need */
    static vex::rotation rotation_sensor;
    static vex::optical optical_sensor;
    static vex::limit limit_switch;
    
    void init();
    void control();

    void two_bar_control();
    void claw_piston_control();
    void lift_to_stage(int stage, int timeout_ms = 2000);

    
private:
    static vex::task claw_macro_task;
    static bool claw_macro_grabbed;
    static vex::task lift_macro_task;
    static int claw_macro_loop();
    static int lift_stage;
    static int lift_macro_loop();
};