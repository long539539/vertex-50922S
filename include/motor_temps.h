#pragma once

/**
 * @brief Starts a background task that shows the temperature (%) of the
 * left lift, right lift, toggle and claw motors on the controller screen.
 * Safe to call more than once, it only starts the task the first time.
 */
void start_motor_temp_display();
