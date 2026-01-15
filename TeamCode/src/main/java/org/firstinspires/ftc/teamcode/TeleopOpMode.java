// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

/*
 * Primary Teleop OpMode for the 2024-2025 INTO THE DEEP competition robot.
 * This OpMode is setup for two-controller (gamepad1 and gamepad2) operation of the robot and
 * does not support any
 */
@TeleOp(name= "Two-controller Teleop", group="Competition")
//@Disabled
public class TeleopOpMode extends OpMode
{
    // Create a RobotHardware object to be used to access robot hardware.
    // All of the methods and properties of the RobotHardware class should be accessed via the
    // "robot.", e.g., "robot.move(x, y, yaw, speed)," etc. All "constants" (static variables)
    // defined in the RobotHardware class must be accessed via the "RobotHardware." prefix, e.g.,
    // "RobotHardware.MOTOR_SPEED_FACTOR_NORMAL," etc.
    // NOTE: This does not apply to ARM_LIMIT_MIN and ARM_LIMIT_MAX, which are accessed via the
    // "robot." prefix, e.g., "robot.ARM_LIMIT_MIN" and "robot.ARM_LIMIT_MAX," because these values
    // are not static and may change when the arm extension motor encoder position is reset.
    RobotHardware robot = new RobotHardware(this);

    // Gamepad objects to save last gamepad state. These can be used to, e.g., compare button states
    // to previous states to tie discrete actions and toggling states to button presses and releases
    // instead "thrashing" on long button presses that span multiple loop() calls.
    Gamepad lastGamepad1 = new Gamepad();
    Gamepad lastGamepad2 = new Gamepad();

    // Declare OpMode members. The elapsed time is one possible value to display in the telemetry
    // data on the driver station.
    final private ElapsedTime runtime = new ElapsedTime();

    // Speed factor for the robot motors. This is to be used as a current setting that can be
    // changed by button presses on the gamepad and then passed to the move() method of the robot
    // hardware class. The speed factor is used to scale the power levels sent to the motors to
    // control the speed of the robot. The speed factor can be set to one of three values:
    //  - RobotHardware.MOTOR_SPEED_FACTOR_NORMAL: Normal safe speed
    //  - RobotHardware.MOTOR_SPEED_FACTOR_PRECISE: Slow (precise) speed for fine control and odometry tracking
    //  - RobotHardware.MOTOR_SPEED_FACTOR_DAVIS: "Sprint" (fast) speed
    // This may also be displayed in telemetry data on the driver station.
    // Evan Button sets the d-pad speed to a faster setting if need be.
    double speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_NORMAL;
    double evanButton = RobotHardware.MOTOR_SPEED_FACTOR_PRECISE;

    int randNum = (int)(Math.random() * 100) + 1;
    String[] quotes =
            {
                "Wire Management, Will", "Shooting 3\'s", "Life is Short, Cookie is Good, We are Robotics",
                "WE ARE ROBOTICS", "Chick-fil-a run?", "Build Team is just a bunch of people hitting things with hammers",
                "Dont hit the gate", "Suddenly getting the flu today", "Will will be with us in spirit", "Get up and walk",
                "Dont stop moving.", "Monstared", "fix it evan",
                "TRAPP!", "Code Issue", "Build Issue", "fly high fry guy", "literal movie",
                "one jellybean...", "Break the 3 - 2!", "0.00006103515% Chance of failure", "FTC please like us!!",
                "And 1!", "All eyes are watching you, no pressure.", "Logan probably wrote this code",
                "lock in.", "DONT OVERSHOOT.", "They killed PlungerBot", "go for it", "=)", "WILL KINNNG", "100% Brain Power", "Simple. Dont Miss", ""
            };

    String famousLastWords = quotes[(int)(Math.random() * quotes.length)]; //Selects a random quote to display on telemetry


    // A hashmap to store the speed factor names for display in telemetry data on the driver station.
    static final private HashMap<Double, String> speedFactorNames;
    static {
        speedFactorNames = new HashMap<>();
        speedFactorNames.put(RobotHardware.MOTOR_SPEED_FACTOR_NORMAL, "Normal");
        speedFactorNames.put(RobotHardware.MOTOR_SPEED_FACTOR_PRECISE, "Precise");
        speedFactorNames.put(RobotHardware.MOTOR_SPEED_FACTOR_DAVIS, "Davis");
    }

    // Flag to indicate that the arm extension motor is in a RUN_TO_POSITION operation. This allows
    // a single button press to extend/retract the arm to a specific position without tying up the
    // thread in the loop() function waiting for the arm to reach the desired position.

    int lastLeftPos = 0;
    int lastRightPos = 0;
    ElapsedTime launcherTimer = new ElapsedTime();

    double shootDist =  .68;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the robot hardware through the RobotHardware class.
        robot.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Hardware Initialized");
        telemetry.addLine();
        telemetry.addLine("lock in.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        // reset the runtime counter
        runtime.reset();

        // ***** What's the current extension of the arm? We need some way to determine what the
        // encoder value is for the arm's extension motor before robot.init() because it is going
        // to be reset to zero on init. *****

        // Setup the initial telemetry display for the driving team captain
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addData("Speed Factor", speedFactorNames.get(speedFactor));



    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {




        // ***** Handle movement control from Gamepad1 *****

        // Set the current speedFactor from button presses on the first gamepad:
        // A: Normal speed
        // B: Precise speed
        // X: Fry speed
        // Y: D-Pad speed setter

        if (gamepad1.a && !lastGamepad1.a) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_NORMAL;
        } else if (gamepad1.x && !lastGamepad1.x) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_DAVIS; // fly high fry guy
        } else if (gamepad1.b && !lastGamepad1.b) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_PRECISE;
        } else if (gamepad1.y) {
            evanButton = speedFactor;
        }

        //else {
        //    System.out.println("Logan probably messed up the code.");
        //}

        // If d-pad input provided, ignore joystick input(s)
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {

            // Move the robot in the direction of the d-pad button pressed at 1/2 precision speed
            if (gamepad1.dpad_up && !lastGamepad1.dpad_up) {
                robot.move(1, 0, 0, evanButton * 0.5);
            } else if (gamepad1.dpad_down && !lastGamepad1.dpad_down) {
                robot.move(-1, 0, 0, evanButton * 0.5);
            } else if (gamepad1.dpad_left && !lastGamepad1.dpad_left) {
                robot.move(0, 1, 0, evanButton * 0.5);
            } else if (gamepad1.dpad_right && !lastGamepad1.dpad_right) {
                robot.move(0, -1, 0, evanButton * 0.5);
            }


        } else {
            // ignore right stick inputs if bumpers are pressed
            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                //rotates the bot based off of set speed.
                if (gamepad2.right_bumper)
                    robot.move(0, 0, -2, evanButton * 0.5);
                else if (gamepad2.left_bumper)
                    robot.move(0, 0, 2, evanButton * 0.5);
            } else {
                // Use left joystick to go forward & strafe, and right joystick to rotate.
                // NOTE: the robot.move() function takes values in FTC coordinate system values, where
                // +x is forward, +y is left, and +yaw is counter-clockwise rotation.
                double axial = -gamepad1.left_stick_y;  // pushing stick forward gives negative value
                double lateral = -gamepad1.left_stick_x;  // pushing stick left gives negative value
                double yaw = -gamepad1.right_stick_x;  // pushing stick left gives negative value
                robot.move(axial, lateral, yaw, speedFactor);





            }
        }

        // ***** Kebob / Launcher from Gamepad2 *****

        //Controls for Player 2 - Shoot Person

            if (!lastGamepad2.dpad_up && gamepad2.dpad_up )
                robot.switchToIndex(0);
            else if (!lastGamepad2.dpad_down && gamepad2.dpad_down)
                robot.switchToIndex(1);
            else if (gamepad2.a)
                robot.shootOn(robot.currLaunchVel);
            else if (gamepad2.b)
                robot.shootOff();

            else if (gamepad2.xWasPressed())
                robot.forwardSizzleSteak(.7);
            else if (gamepad2.xWasReleased())
                robot.sizzleSteakOff();
            else if (gamepad2.yWasPressed())
                robot.reverseSizzleSteak(.7);
            else if (gamepad2.yWasReleased())
                robot.sizzleSteakOff();
            else if (gamepad2.left_trigger >= 0.5) {
                robot.reverseLauncher(.6);
            }
        telemetry.addLine("-----------Player 1-----------");
        telemetry.addData("Speed Factor", speedFactorNames.get(speedFactor));
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addLine("-----------Player 2-----------");
        telemetry.addData("Launch Dist: ", robot.getShootDist());
        telemetry.addData("Index (I): ", robot.getIndex());
        telemetry.addLine("-----------Extra-----------");
        telemetry.addLine(famousLastWords);
        telemetry.addData("Random Number of the Day: ", randNum);
        telemetry.update();

    }





    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Stop the robot
        robot.stop();

        telemetry.addData("Status", "Stopped. Total Runtime: (%s)", runtime.toString());
    }
}
