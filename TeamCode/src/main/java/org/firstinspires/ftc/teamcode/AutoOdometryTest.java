// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * A Linear OpMode with a simple linear route and turns for testing mid-level drive functions
 * in the RobotHardware class along with validation of odometry calibration and PID coefficient
 * settings.
 */

@TeleOp(name="Autonomous Drive/Odometry Test", group="Test")
//@Disabled

public class AutoOdometryTest extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    final ElapsedTime runtime = new ElapsedTime();
    Gamepad lastGamepad1 = new Gamepad(); // save last gamepad state.

    @Override
    public void runOpMode() {

        // initialize all the hardware, including vision portal
        // NOTE: This resets the odometry counters to zero
        robot.init(true);

        // Update the status in telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Start the timer
        runtime.reset();

        // Perform autonomous motion commands
        // NOTE: Each mid-level drive command in the RobotHardware class has it's own loop
        // with a check for opModeIsActive(). But check here before each command to avoid
        // potentially leaving the robot in motion.

        // NOTE: These steps assume the robot is initially positioned in the test square
        // facing parallel to the red alliance wall.

        // Move forward 2500 mm and display odometry telemetry
       // if (opModeIsActive()) {
         //   telemetry.addData("Running forward(2000) call...", false);
           // telemetry.update();
           // robot.forward(2000, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            //showTelemetryAndWait();
        //}

        // Strafe left 2000 mm
        if (opModeIsActive()) {
            robot.strafe(2000, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            showTelemetryAndWait();
        }

        // Rotate clockwise 90 degrees
        if (opModeIsActive()) {
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            showTelemetryAndWait();
        }

        // Move forward 2000 mm
        if (opModeIsActive()) {
            robot.forward(2000,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            showTelemetryAndWait();
        }

        // Rotate clockwise 90 degrees
        if (opModeIsActive()) {
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            showTelemetryAndWait();
        }

        // Move forward 2500 mm - Robot should end up back in the test square facing the opposite
        // direction from the start position. :)
        if (opModeIsActive()) {
            robot.forward(2000,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            showTelemetry();
        }

        // Stop the robot - just in case
        robot.stop();

        // Enter a teleop period to allow for manual invocation of mid-level drive functions
        while (opModeIsActive()) {

            // call mid-level motion function to Move the robot about a foot in the direction
            // of the d-pad button pressed or rotate 90 degrees in the direction of the bumper
            // pressed. This is for testing odometry and PID tuning.
            if(gamepad1.dpad_up)
                robot.forward(300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.dpad_down)
                robot.forward(-300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.dpad_left)
                robot.strafe(300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.dpad_right)
                robot.strafe(-300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.left_bumper)
                robot.turn(Math.PI / 2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.right_bumper)
                robot.turn(-Math.PI / 2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

            // Move the robot to the test square facing the blue alliance wall
            // NOTE: Use the Drivetrain test/calibration opmode to move the robot into this position
            // and then pull the right trigger to read the x, y, and heading position along with the
            // tag ID to supply to the moveToPositionUsingAprilTag() function.



            // Display odometry telemetry
            showTelemetry();

            // Save the current gamepad state
            lastGamepad1.copy(gamepad1);
        }

        // stop the robot (just in case)
        robot.stop();

        // Display the time elapsed and final field position in telemetry
        telemetry.addData("Status", "Stopped. Run Time: %s", runtime.toString());
        telemetry.update();
    }

    void showTelemetry() {

        // Display odometry telemetry
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addData("Odometry", "X: %.1f  Y: %.1f  Theta: %.3f",
                robot.getOdometryX(), robot.getOdometryY(), robot.getOdometryHeading());
        telemetry.update();
    }
    void showTelemetryAndWait() {

        // Call the showTelemetry method to display telemetry data
        showTelemetry();

        // Wait for X button to be pressed to continue
        while (!gamepad1.x && opModeIsActive())
            idle();
    }
}
