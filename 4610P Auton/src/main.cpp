#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "liblvgl/lv_init.h"
#include "pros/apix.h" // IWYU pragma: keep
#include <string>


// =========== VARIABLES ============ //
int autonNumber = 0;
int maxAuton = 10;
bool autonSelected = false;
const char* autonNames[10] = {
    "Red Right",
    "Red Left",
    "Blue Right",
    "Blue Left",
    "Skills",
    "Test 1",
    "Test 2",
    "Test 3",
    "Test 4",
    "Unused"
};

// =========== END VARIABLES ============ //


// =========== DEVICES ============= //
pros::MotorGroup left_motors({-2, 5, -19}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({9,-10, 12 }, pros::MotorGearset::blue); // right motors use 200 RPM cartridges
pros::Imu imu(4);
pros::Rotation rotation_sensor(1);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros:: Motor intake(-6, pros::MotorGears::blue); //Creates a motor on port 3 with blue gear set
pros:: Motor decider(-7, pros::MotorGears::green); //Creates a motor on port 3 with blue gear set
pros:: Motor top(-16, pros::MotorGears::green); //Creates a motor on port 3 with blue gear set
pros::adi::DigitalOut stopper('F');
pros::adi::DigitalOut tongue('A');
lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_2, 0);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                     nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                     nullptr, // horizontal tracking wheel 1
                     nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                     &imu // inertial sensor
);
// =========== END DEVICES ============= //



// ========== LEMLIB SETTINGS ============ //

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              9, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// ========== END LEMLIB SETTINGS ============ //




// ========== FUNCTIONS ============ //
void initialize() {
    lv_init();
    pros::lcd::initialize();
    chassis.calibrate();

}

void on_left_button() {
    if (!autonSelected) {
        autonNumber--;
        if (autonNumber < 0) {
            autonNumber = maxAuton - 1;
        }
    }
}
void on_right_button() {
    if (!autonSelected) {
        autonNumber++;
        if (autonNumber >= maxAuton) {
            autonNumber = 0;
        }
    }
}
void on_center_button() {
    autonSelected = !autonSelected;
}


void updateAutonDisplay() {
    int idx = autonNumber;
    if (idx < 0) idx = 0;
    if (idx >= maxAuton) idx = maxAuton - 1;
    std::string line0 = "Auton " + std::to_string(idx) + ": " + autonNames[idx];
    std::string line1 = autonSelected ? "*** SELECTED ***" : "< choose / lock >";
    pros::lcd::set_text(0, line0);
    pros::lcd::set_text(1, line1);
}

void autonDisplayTask(void*) {
    while (true) {
        updateAutonDisplay();
        pros::delay(200);
    }
}

void startAutonDisplay() {
    pros::lcd::initialize();
    pros::lcd::register_btn0_cb(on_left_button);
    pros::lcd::register_btn1_cb(on_center_button);
    pros::lcd::register_btn2_cb(on_right_button);
    static pros::Task t(autonDisplayTask, (void*)"auton_disp");
}
void disabled() {
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

}



void competition_initialize() {
/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
    
startAutonDisplay();


}


void redRight() {
	// 4 low, 3 high
    chassis.setPose(-48.5, -15, 90);
    intake.move(127);
    decider.move(60);
    top.move(60);
    stopper.set_value(0);
	chassis.turnToHeading(103, 500);
    chassis.moveToPoint(-20, -24, 1000, {true, 60, 10}); // gets 3 blocks
    pros::delay(500);
	chassis.turnToPoint(-12, -14, 1000, {true,AngularDirection::AUTO,40, 5}); // turns to low goal
	pros::delay(500);
	chassis.moveToPoint(-12, -14, 2500, {true, 60, 5}); // aligns with low goal
	pros::delay(750);
    intake.move(-127);
    decider.move(-127);
    top.move(-127);
    pros::delay(2500);
	intake.move(127);
	decider.move(127);
    top.move(127);
	chassis.moveToPoint(-20, -22.5, 1250, {false, 80, 10}); // backs up
	chassis.turnToPoint(-46, -48, 1000);
    tongue.set_value(1);
	chassis.moveToPoint(-46, -48, 1000, {true, 80, 10});; // positions to match loader
	pros::delay(500);
    chassis.turnToPoint(-56, -48, 1000, {true,AngularDirection::AUTO,40, 5});
	chassis.moveToPoint(-56, -48, 1500, {true, 80, 30}, false); // goes into match loader
	pros::delay(1500);
	intake.move(0);
	decider.move(0);
    top.move(0);
	stopper.set_value(1);
	chassis.moveToPoint(-26, -48, 1500, {false, 80, 10}); // aligns with high goal
	pros::delay(750);
	intake.move(127);
	decider.move(127);
    top.move(127);
	pros::delay(2250);
	intake.move(0);
	decider.move(0);
	top.move(0);
	chassis.moveToPoint(-38, -48, 1000); // backs up
	stopper.set_value(0);
	pros::delay(500);
	chassis.moveToPoint(-26, -48, 1000, {false, 127, 127}); // re-rams to push blocks further
    }
    


void autonomous() {
    switch (autonNumber) {
        case 0:
            redRight();
        case 1:
            // redLeft();
        case 2:
            // blueRight();
        case 3:
            // blueLeft();
        
      
    }
}

void opcontrol() {
  while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        // High goal
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
          intake.move(127);
          top.move(127);
          decider.move(127);
        // bottom goal
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
          intake.move(-127);
          top.move(-127);
          decider.move(-127);
        // middle goal
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
          intake.move(-127);
          top.move(-127);
          decider.move(127);
        } else {
          intake.move(0);
          top.move(0);
          decider.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            tongue.set_value(1);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            tongue.set_value(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            stopper.set_value(1);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            stopper.set_value(0);
        }

        // delay to save resources
        pros::delay(25);
    }}