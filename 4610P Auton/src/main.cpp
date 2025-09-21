#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
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

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
// lateral PID controller
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
// angular PID controller
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

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
    chassis.calibrate();

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
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
    /*
	If have time, go back to match loader and unload blue blocks and spit them out front
	chassis.moveToPoint(-56.25, -47, 60);
	intake.move(127);
	decider.move(127);
    top.move(127);
	stopper.set_value(1);
    */
    }
    


void autonomous() {
redRight();
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
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