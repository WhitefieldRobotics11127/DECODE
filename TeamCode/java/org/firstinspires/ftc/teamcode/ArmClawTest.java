package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode is for testing and calibrating the arm rotation motor, viper-slide extension motor, and
 * gripper servo on the 2024-2025 INTO THE DEEP competition robot.
 *
 * This OpMode is teleop and in the "Test" group.
 *
 * NOTE: This OpMode contains a servo zeroing feature.  When the X button is pressed, the claw
 * servo will reset to the CLOSED position. This should be done BEFORE installing the right gripper
 * arm on the claw body to ensure that the named positions and limits in the RobotHardware class are
 * correct.
 */

@TeleOp(name="Test and Calibrate Arm and Claw", group="Test")
//@Disabled
public class ArmClawTest extends OpMode
{
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // objects to save last gamepad state
    Gamepad lastGamepad1 = new Gamepad();
    Gamepad lastGamepad2 = new Gamepad();

    // Declare OpMode members.
    final private ElapsedTime runtime = new ElapsedTime();

    // for tracking position based arm extension operations
    private boolean inArmExtensionOperation = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // move the servo to zero position if X button is pressed
        // NOTE: used for if the A button is pressed
        if (gamepad1.x) {
            robot.closeClaw(false);
        }
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        // set the initial position of the claw
        robot.closeClaw(false);

        // Tell the driver that the OpMode is running.
        telemetry.addData("Status", "Running...");
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // Right trigger controls the claw servo
        double servoPosition;
        if (gamepad1.right_trigger > 0.4)
            robot.closeClaw((gamepad1.left_trigger > 0.4));
        else
            robot.openClaw((gamepad1.left_trigger > 0.4));

        // if the arm extension motor is busy extending arm to a position, check progress
        // and reset flag if finished
        if (inArmExtensionOperation) {
            if (!robot.isArmExtensionBusy()) {
                robot.stopArmExtension();
                inArmExtensionOperation = false;
            }
        }
        else {

            // Reset the encoder if the A button is pressed
            if (gamepad1.a) {
                robot.resetArmLimits();
            }

            // If a particular position for the arm extension is desired, set it using the
            // setArmExtension method
            if (gamepad1.left_bumper && !lastGamepad1.left_bumper ||
                    gamepad1.right_bumper && !lastGamepad1.right_bumper ||
                    gamepad1.dpad_up && !lastGamepad1.dpad_up ||
                    gamepad1.dpad_down && !lastGamepad1.dpad_down) {

                int pos = robot.getArmExtension();
                // bumpers move to the limits, dpad up and down move the arm extension by 300 ticks
                if (gamepad1.left_bumper)
                    pos = 0;
                else if (gamepad1.right_bumper)
                    pos = RobotHardware.ARM_EXTENSION_LIMIT;
                else if (gamepad1.dpad_up)
                    pos += 300;
                else if (gamepad1.dpad_down)
                    pos -= 300;

                // Set the arm extension to the specified position
                robot.setArmExtension(pos);
                inArmExtensionOperation = true;
            }

            // otherwise, get the arm movement from the left stick
            else {

                // Left stick Y controls the arm extension.
                robot.extendArm(-gamepad1.left_stick_y);
            }
        }

        // Right stick Y controls the arm rotation.
        // NOTE: The rotateArm() method takes positive values to lower the arm and negative values
        // to raise the arm. This is so that the power applied for the rotation tracks with the
        // values returned from the rotation position sensor (potentiometer).
        robot.rotateArm(-gamepad1.right_stick_y);

        // Save the current gamepad states
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running...");
        telemetry.addData("Extension", "Position: %d", robot.getArmExtension());
        telemetry.addData("Extension", "Busy: %b", robot.isArmExtensionBusy());
        telemetry.addData("Rotation", "Position: %.2f", robot.getArmRotation());
        telemetry.addData("Rotation", "Voltage: %.3f", robot.getArmRotationSensorVoltage());
        telemetry.addData("Rotation", "Power %.2f", robot.getArmRotationMotorPower());
        telemetry.addData("Claw", "Position: %.2f", robot.getClawPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // stop any arm extension motor movement
        robot.stopArmExtension();
        robot.stopArmRotation();


        // Tell the driver that the OpMode has stopped.
        telemetry.addData("Status", "Stopped");
    }
}