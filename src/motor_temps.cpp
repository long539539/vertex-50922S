#include "vex.h"
#include "motor_temps.h"

// Shows motor temperature (%) on the controller screen:
//   Row 1: Lift L: 45% R: 47%
//   Row 2: Togl: 30% Claw: 28%
//   Row 3: "Motors OK" or "HOT! <motor>" when a motor reaches 55 C
//
// Uses the motors you already made in robot-config.cpp:
//   Assembly::cascade_motors -> [0] = right_lift (PORT8), [1] = left_lift (PORT13)
//   Assembly::toggle_motor   (PORT19)
//   Assembly::claw_motor     (PORT10)

static vex::task temp_task;
static bool temp_task_started = false;

// V5 motors start cutting their power at about 55 C
static const double HOT_CELSIUS = 55.0;

static int temp_pct(vex::motor& m) {
    return (int)m.temperature(vex::percentUnits::pct);
}

static int temp_display_loop() {
    char line[32];
    int row = 0;

    while (true) {
        // Do not draw over the auton selector or while auton is running
        if (control_disabled() || Competition.isAutonomous()) {
            vex::task::sleep(200);
            continue;
        }

        std::vector<mik::motor*>& lift = Assembly::cascade_motors.getMotors();
        if (lift.size() < 2) {
            vex::task::sleep(500);
            continue;
        }
        vex::motor& right_lift = *lift[0];
        vex::motor& left_lift  = *lift[1];
        vex::motor& toggle     = Assembly::toggle_motor;
        vex::motor& claw       = Assembly::claw_motor;

        // The controller screen is slow, so only update one row at a time
        if (row == 0) {
            snprintf(line, sizeof(line), "Lift L:%3d%% R:%3d%%",
                     temp_pct(left_lift), temp_pct(right_lift));
        } else if (row == 1) {
            snprintf(line, sizeof(line), "Togl:%3d%% Claw:%3d%%",
                     temp_pct(toggle), temp_pct(claw));
        } else {
            const char* hot_name = nullptr;
            double hottest = HOT_CELSIUS;
            vex::motor* motors[4] = { &left_lift, &right_lift, &toggle, &claw };
            const char* names[4]  = { "Lift L", "Lift R", "Toggle", "Claw" };
            for (int i = 0; i < 4; i++) {
                double c = motors[i]->temperature(vex::temperatureUnits::celsius);
                if (c >= hottest) {
                    hottest = c;
                    hot_name = names[i];
                }
            }
            if (hot_name) snprintf(line, sizeof(line), "HOT! %s", hot_name);
            else          snprintf(line, sizeof(line), "Motors OK");
        }

        Controller.Screen.setCursor(row + 1, 1);
        Controller.Screen.print("%-19s", line);  // pad with spaces to erase old text

        row = (row + 1) % 3;
        vex::task::sleep(100);
    }
    return 0;
}

void start_motor_temp_display() {
    if (temp_task_started) return;
    temp_task_started = true;
    temp_task = vex::task(temp_display_loop);
}
