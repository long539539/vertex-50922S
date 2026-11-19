#include "vex.h"
#include "motor_temps.h" 

task UI;

// If you want to disable Brain UI, cut wireless upload times to ~7s
// Go into `makefile` and set `FAST_COMPILE = 1`
static int run_UI() {
    UI_init();
    UI_render();
    return 0;
}

void pre_auton() {
    for (int i = 0; i < 10 && !keep_vexos_timer; i++) {
        keep_vexos_timer = Competition.isFieldControl() || Competition.isCompetitionSwitch();
        task::sleep(50);
    }
    init({
        // Running the auton selector on the controller disables user control,
        // press X on the controller to close auton selector and enable user control
        .enable_controller_selector = true,
        
        // Warns disconnected motors and chassis devices. 
        // Displayed on `Error Data` on Config Tab
        .check_disconnected_devices = true
        
    });

    default_constants();
    assembly.lift_rotation.resetPosition();  // lift must be all the way down at program start
    assembly.optical_task = vex::task(Assembly::optical_loop);  // runs through auton and driver
    UI = task(run_UI);
}

void auton(void) {
    UI.stop();
    auton_scr->start_auton();
    
}

void user_control(void) {
    while (chassis.calibrating) { task::sleep(50); }
    start_motor_temp_display();

    // How you want your drivetrain to stop during driver
    chassis.set_brake_type(brakeType::coast);

    while (true) {
        if (!control_disabled()) {
            // Add your user control code here
            
            // if (!assembly.anti_tip_step()) {
            //     chassis.control(drive_mode::SPLIT_ARCADE);
            // }
            chassis.control(drive_mode::SPLIT_ARCADE);
            assembly.control();
        }
        task::sleep(5);
    }
}

int main() {
    Competition.autonomous(auton);
    Competition.drivercontrol(user_control);

    pre_auton();

    while (true) {
        task::sleep(100);
    }
}
