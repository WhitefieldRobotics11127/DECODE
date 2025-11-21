// This file not meant for redistribution - copyright notice removed
/*
 * This class provides a hardware abstraction layer for WA Robotics INTO THE DEEP competition robot.
 *
 * This RobotHardware class is modified from the FTC SDK example code and was rebuilt from the
 * ground up in the 2024-2025 season in conjunction with a new crop of students and mentors.
 *
 * Support for Mecanum (Omni) drivetrain, odometry, IMU, and vision processing is included, which
 * can (hopefully) be reused from year to year. Support for INTO THE DEEP game-specific hardware,
 * such as the Viper-Slide arm and the servos for the claw are season-specific but can serve as
 * examples.
 *
 * Also included in this class are methods and classes for performing autonomous motion using
 * odometry. Odometry calculations and forward, strafe, and turn functions are included.
 *
 * Many parameter values must be tuned for the specific robot and competition, and these are noted
 * in the comments.
 *
 * To simplify all calculations, all lengths are in MM and all angles are in radians. The NWU
 * (North, West, and Up) coordinate frame is used for robot axes, with the +X-axis forward, the
 *  +Y-axis to the left, the +Z-axis up, and +Yaw is counterclockwise.
 */

package org.firstinspires.ftc.teamcode;

/*
 * For the most part, imports are managed by the Android Studio IDE through the "Auto Import"
 * feature setting (under File->Settings, drill down to Editor>General>Auto Import), but may
 * occasionally need to be cleaned up to remove unused imports.
 */
import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.util.Range.clip;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Hardware abstraction class for WA Robotics DECODE competition robot
 */
public class RobotHardware {

    /* ----- Public constants (so they can be used by the calling OpMode) ----- */
    // Allow drivetrain to operate in different at different, selectable "speeds"
    /**
     * Normal speed for movement commands.
     */
    public static final double MOTOR_SPEED_FACTOR_NORMAL = 0.6;
    /**
     * "Sprint" speed for movement commands.
     */
    public static final double MOTOR_SPEED_FACTOR_DAVIS = 1.0;
    /**
     * Slower speed for movement commands to allow for precise odometry tracking.
     */
    public static final double MOTOR_SPEED_FACTOR_PRECISE = 0.35;
    /**
     * Separate speed for Autonomous movement commands to use.
     */
    public static final double MOTOR_SPEED_FACTOR_AUTONOMOUS = 0.44;

    // Allowable limits for arm rotation
    // NOTE: These are [0, 1) within voltage rage of potentiometer
    /**
     * Minimum safe rotational position for arm.
     * NOTE: Fully upright arm is the "minimum" position (within 0.0 to 1.0 range) in order to 
     * directly track with position sensor (potentiometer) values. 
     */
    //public static final double ARM_ROTATION_MIN = 0.0;
    /**
     * Maximum safe rotational position for arm.
     * NOTE: Fully rotated down is the "maximum" position (within 0.0 to 1.0 range) in order to 
     * directly track with position sensor (potentiometer) values. 
     */
    //public static final double ARM_ROTATION_MAX = 0.40;

    // Limits for extension of arm.
    /**
     * Encoder position for fully extended viper slide (arm) when horizontal.
     * This value is set lower than the physical extension limit to fit into the 42" lateral reach
     * limit in the FTC rules. Use the extendArmToLimit() method to extend the arm to the maximum
     * allowable limit based on the arm rotation position.
     * NOTE: This assumes that the fully retracted is currently set as 0 encoder value.
     */
    //public static final int ARM_EXTENSION_LIMIT = 2640;
    /**
     * Encoder position for fully extended viper slide (arm) when vertical.
     * This value is the maximum physical extension limit and is not subject the 42" limit of the
     * FTC rules when the arm is vertical. This can be used, e.g., when reaching for the the top
     * bucket on the INTO THE DEEP field. Use the extendArmToLimit() method to extend the arm to the
     * maximum allowable limit based on the arm rotation position.
     * NOTE: This assumes that the fully retracted is currently set as 0 encoder value.
     */
    //public static final int ARM_EXTENSION_LIMIT_FULL = 2915;

    // Servo positions for claw
    // NOTE: these are [0, 1) within the min and max range set for the servo
    /** Servo position for open claw. */
    //public static final double CLAW_SERVO_OPEN = 0.5;
    /** Servo position for closed claw. */
    //public static final double CLAW_SERVO_CLOSE = 0.1;
    /** Servo position for widest opening of claw. */
    //public static final double CLAW_SERVO_OPEN_WIDE = 0.7;
    /**
     * Servo position for tightly gripping claw.
     */
    //static final double CLAW_SERVO_CLOSE_GRIP = 0.0;

    /* ----- Member variables (private so hidden from the calling OpMode) ----- */

    /*
     * Parameter values for drivetrain motors.
     * Even though all robots will likely use four-motor Mecanum (Omni) drivetrains, these constants
     * may have to be modified each year based on the robots mechanical configuration, weight, and
     * performance and whether the drive motors use encoders or not.
     */
    // Correction factors for individual motors to account for mechanical differences
    // NOTE: If the robot is not driving straight, adjust these values to correct the issue.
    // NOTE: these values may not be needed if all motors are using encoders and their run modes
    // are set to RUN_USING_ENCODER
    static final double OMNI_CORRECTION_LEFT_FRONT = 1.0; // Correction factor for left front motor
    static final double OMNI_CORRECTION_RIGHT_FRONT = 1.0; // Correction factor for right front motor
    static final double OMNI_CORRECTION_LEFT_BACK = 1.0; // Correction factor for left back motor
    static final double OMNI_CORRECTION_RIGHT_BACK = 1.0; // Correction factor for right back motor

    /*
     * Parameter values for odometry calculations.
     * These are used in the calculation of the current position (relative movement and field
     * coordinates). The values are initially set from physical measurements of the robot but should
     * be tweaked for accuracy from testing (e.g., spin test and/or strafe/curve testing).
     */
    static final int DEADWHEEL_LEFT_DIRECTION = 1; // Allows for adjustment of + direction of left encoder - should be installed front to back
    static final int DEADWHEEL_RIGHT_DIRECTION = -1; // Allows for adjustment of + direction of right encoder - should be installed front to back
    static final int DEADWHEEL_AUX_DIRECTION = -1; // Allows for adjustment of + direction of aux encoder - should be installed left to right
    // The following values were last calibrated 2024-12-23
    static final double DEADWHEEL_MM_PER_TICK = 0.07486; // MM per encoder tick (initially calculated 48MM diameter wheel @ 2000 ticks per revolution)
    static final double DEADWHEEL_FORWARD_OFFSET = -106.0; //forward offset (length B) of aux deadwheel from robot center of rotation in MM (negative if behind)
    // 2025-01-15 Changed trackwidth from 306.7 to 305 to account for consistent over-rotation
    static final double DEADWHEEL_TRACKWIDTH = 305; // distance (length L) between left and right deadwheels in MM

    /*
     * Constants for autonomous motion routines.
     * These may require a lot of tweaking.
     */
    // Tolerance values for closed-loop controllers for use in translate and rotate commands
    // NOTE: These were made public on 2025-01-27 for use in autonomous motion routines in figuring
    // out how to reposition based on AprilTag poses and/or distance sensors.
    public static final double X_POSITION_TOLERANCE = 10; // Tolerance for position in MM (~ 1/2 inch)
    public static final double Y_POSITION_TOLERANCE = 10; // Tolerance for position in MM (~ 1/2 inch)
    public static final double HEADING_TOLERANCE = 0.035; // Tolerance for heading in radians (~2 degrees)
    //static final double X_CONTROLLER_DEADBAND = 3.175; // Deadband range for X power calculation. Should be less than MOVE_POSITION_TOLERANCE
    //static final double Y_CONTROLLER_DEADBAND = 3.175; // Deadband range for Y power calculation. Should be less than MOVE_POSITION_TOLERANCE
    //static final double YAW_CONTROLLER_DEADBAND = 0.01; // Deadband range for Yaw power calculation. Should be less than HEADING_TOLERANCE
    // What happens if we use 0 deadband values? Don't the motors already have a deadband built in?
    static final double X_CONTROLLER_DEADBAND = 0;
    static final double Y_CONTROLLER_DEADBAND = 0;
    static final double YAW_CONTROLLER_DEADBAND = 0;


    // PID gain values for each of the three closed-loop controllers (X, Y, and heading). These need
    // to be calibrated:
    //  - For proportional (Kp) - start with reasonable distance (error) (in mm) where the robot should start
    // to slow down while approaching the destination and take the inverse. Then adjust up until the robot
    // regularly oscillates around the target position.
    // - Once Kp is set, increase the Kd value from zero until the end behavior stabilizes.
    // - We will likely not use Ki values.
    static final double X_CONTROLLER_KP = 0.0050; // Proportional gain for axial (forward) position error - start slowing down at 250 mm (~ 10 in.)
    static final double X_CONTROLLER_KD = 0.0; // Derivative gain for axial (forward) position error
    static final double X_CONTROLLER_KI = 0.0; // Integral gain for axial (forward) position error
    static final double Y_CONTROLLER_KP = 0.0067; // Proportional gain for lateral (strafe) position error - start slowing down at 150 mm (~ 6 in.)
    static final double Y_CONTROLLER_KD = 0.0; // Derivative gain for lateral (strafe) position error
    static final double Y_CONTROLLER_KI = 0.0; // Integral gain for lateral (strafe) position error
    static final double YAW_CONTROLLER_KP = 1.637; // Proportional gain for yaw (turning) error - start slowing down at 0.6109 radians (~ 35 degrees)
    static final double YAW_CONTROLLER_KD = 0.0; // Derivative gain for yaw (turning) error
    static final double YAW_CONTROLLER_KI = 0.0; // Integral gain for yaw (turning) error

    /*
     * Parameter values for arm (Viper-slide) and claw.
     */
    // Arm rotation position where the arm is considered "vertical" for selectively applying the
    // two limit values ARM_EXTENSION_LIMIT and ARM_EXTENSION_LIMIT_FULL to the arm extension motor.
    //static final double ARM_ROTATION_VERTICAL = 0.22;

    // Limit the power to the rotation motor to prevent damage to the arm. This needs to be calibrated.
    //static final double ARM_ROTATION_POWER_LIMIT_FACTOR = 0.7; // Factor to limit power to arm rotation motor

    // Tolerances and proportional gain values for arm rotation position controller. These need to be calibrated.
    //static final double ARM_ROTATION_DEADBAND = 0.006; // Deadband range for arm rotation position
    //static final double ARM_ROTATION_TOLERANCE = 0.012; // Tolerance for arm rotation position
    //static final double ARM_ROTATION_KP = 25.0; // Proportional gain for arm rotation position error

    // Maximum voltage from arm rotation potentiometer
    // NOTE: this is set in init() from the getMaxVoltage() method on the potentiometer and
    // utilized to calculate an arm rotation position in the [0, 1)
    // range.
    //double ARM_ROTATION_MAX_VOLTAGE;

    // Limit the power to the extension motor to prevent damage to the arm. This needs to be calibrated.
    //static final double ARM_EXTENSION_POWER_LIMIT_FACTOR = 0.85; // Factor to limit power to arm extension motor

    // Tolerances and proportional gain values for arm extension position controller. These need to be calibrated.
    //static final int ARM_EXTENSION_DEADBAND = 25; // Deadband range for arm extension position in ticks (1/4 turn)
    //static final double ARM_EXTENSION_KP = 0.00333; // Proportional gain for arm extension position error

    // Servo position limits for claw
    //static final double CLAW_SERVO_RANGE_MIN = 0.0;
    //static final double CLAW_SERVO_RANGE_MAX = 0.5;

    /*
     * Constants for vision (AprilTag) processing.
     */
    // Position and orientation of camera(s) on robot for AprilTag detection and field position
    // calculation. These values relate the position of the center of the camera lens relative to
    // the center of rotation of the robot at field height on three FTC-defined robot axes: +y
    // forward, +x right, and +z upward, with orientation being: yaw about the z-axis, pitch about
    // the x-axis, and roll about the y-axis.
    // NOTE: This definition of the robot's axes are different than the NWU robot axes (+x forward,
    // +y left) we use for motion that we got from gm0 and WPILib, so in order to make the heading
    // value of the returned field pose correct for our robot, we need to skew the yaw by -90
    // degrees in the orientation. Thus for a camera pointed left, the yaw is 0, pointing forward is
    // -90, pointing right is 180 degrees, and pointing backward is 90 degrees. Also, a pitch of 0
    // would have the camera pointing straight up, so we need to set the pitch to -90 degrees
    // (rotation about the x-axis), meaning the camera is horizontal.
    // NOTE: Camera 1 is right-side camera and Camera 2 is left-side camera
    private final Position cam1Position = new Position(DistanceUnit.MM,
            205.0, -36.0, 105.0, 0);
    private final YawPitchRollAngles cam1Orientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0);
    private final Position cam2Position = new Position(DistanceUnit.MM,
            -205.0, -36.0, 105.0, 0);
    private final YawPitchRollAngles cam2Orientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /*
     * Hardware objects for current robot hardware.
     * Any functionality or properties of any of these objects needed by OpModes will need to be
     * exposed through methods added to this class (thus the "abstraction" layer).
     */
    // NOTE: We should use the DCMotorEx class for all motors connected to a REV Control Hub or
    // REV Expansion Hub whether or not we are using RUN_USING_ENCODERS or other extended
    // functionality because the built-in REV motor controllers support all the functionality of
    // the DCMotorEx class.
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;  //  Motors for Mecanum drive
    private DcMotorEx rightDeadwheelEncoder, leftDeadwheelEncoder, auxDeadwheelEncoder; // Encoders (deadwheels) for odometry
    private DcMotorEx kebobMotor; // Motor for "Kebob" motor
    private DcMotorEx launcherMotor; // Motors for the launcher motor
    private AnalogInput armRotationPositionSensor; // Potentiometer for arm rotation position
    private Servo lockServo; // Servo for the lock-in-place
    private VisionPortal visionPortal; // Used to manage the camera input and activation of video processors.
    private WebcamName webcam1, webcam2; // For identifying webcam(s)

    // NOTE: Because we want the AprilTag processor to return field position, the precise "pose"
    // of the camera(s) on the robot has to be provided. However, the camera position is set through
    // parameters of the AprilTag processor and not connected directly to the camera, so we need a
    // separate AprilTag processor for each camera. Having separate AprilTag processors also allows
    // us to (theoretically) perform AprilTag detection through multiple cameras simultaneously.
    // To use AprilTag detection, the code has to both set the camera input in the VisionPortal to
    // the desired camera and activate the corresponding AprilTag processor.
    AprilTagProcessor aprilTagCam1, aprilTagCam2; // Provides AprilTag detection through a specific camera.

    //private IMU imu; // IMU built into Rev Control Hub

    private DistanceSensor frontDistanceSensor, rearDistanceSensor; // Distance sensors for detecting distance to wall or structure in front of robot

    /*
     * Variables for tracking robot state
     */
    // last read odometry deadwheel encoder positions - used to calculate encoder deltas since last
    // call to updateOdometry()
    private int lastRightEncoderPosition, lastLeftEncoderPosition, lastAuxEncoderPosition;

    // translated x, y, and heading odometry counters in mm since last reset
    // NOTE: these are updated by the updateOdometry() method and used for simple movement commands
    // (forward, strafe, turn) as well as calculating field position.
    private double xOdometryCounter, yOdometryCounter, headingOdometryCounter;

    // Current robot position (x,y, heading) in field coordinate system
    // NOTE: this is updated by the updateOdometry() method and used for translation and/or rotation
    // to field coordinates
    //private Pose2D currentFieldPosition = new Pose2D(DistanceUnit.MM, 0,0,AngleUnit.RADIANS,0);

    // keep a reference to the calling opmode so that we have access to hardwareMap and other
    // properties and statuses from the running opmode.

    // flag to allow for suspension of arm extension limits. This is used to allow the viper slide
    // to be re-homed and the limits to be reset, particularly after the transition from autonomous
    // to teleop when the robot is reinitialized.
    //private boolean armExtensionLimitsSuspended = false;

    private final OpMode myOpMode;

    /**
     * Constructor allows calling OpMode to pass a reference to itself.
     *
     * @param opmode the OpMode that is creating this RobotHardware instance
     */
    public RobotHardware(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * @param vision true if vision processing is needed, false otherwise
     */
    public void init(boolean vision) {

        // Define Mecanum drivetrain hardware instance variables
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "leftfront_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightfront_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "leftback_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightback_drive");

        // Set the direction, braking, and run mode for each motor 
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD); // based on which way the motor is mounted
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // don't allow overrun
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // increased accuracy and balance from controls

        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Make sure robot it not moving (power to zero and/or stop commands), and initialize
        // encoders
        // NOTE: There's a lot of ambiguity in the documentation and the forums of whether the motor
        // power will be set to 0 when setting the mode to STOP_AND_RESET_ENCODER and whether the
        // mode will change back to RUN_USING_ENCODER after the encoders are reset or not. For both
        // these questions, the the documentation basically says "some motor controllers yes, some
        // motor controllers no." Also, there are questions of whether a short sleep is needed
        // (e.g., 100ms) after setting the mode to STOP_AND_RESET_ENCODER to allow the encoders to
        // reset before setting the mode back to RUN_USING ENCODER.
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setPower(0.0);

        // ????? What do we need here (if anything) to ensure that the encoders are reset before we
        // set the run mode back to RUN_USING_ENCODER? A Thread.sleep() call, a myOpMode.wait()
        // call, a myOpMode.idle() call (only for Linear OpModes), etc.?
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // may not be needed
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Define odometry encoder hardware instance variables
        rightDeadwheelEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_right");
        leftDeadwheelEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_left");
        auxDeadwheelEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_aux");

        // Reset the encoder values
        rightDeadwheelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadwheelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        auxDeadwheelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        // Define launcher hardware instance variables
        // NOTE: ****** The rotation motor uses the same motor port as the auxiliary deadwheel encoder
        kebobMotor = auxDeadwheelEncoder; // Use the same hardware mapping as the auxiliary odometry encoder
        launcherMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "launcher_motor");



        // Initialize settings for launcher and kebob motor
        launcherMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        kebobMotor.setDirection(DcMotorEx.Direction.REVERSE);
        kebobMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        kebobMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reset the encoder count to zero and make sure the motor is stopped
        // NOTE: the viper slide should be fully retracted before "Start" is pressed
        //launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //launcherMotor.setPower(0.0);

        // Set the run mode to RUN_WITHOUT_ENCODER
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Initialize settings for arm motor
        // NOTE: Run the motor in forward direction so that power to the motor is positive when
        // tilting down. This allows the direction of rotation to track with the direction of the
        // rotation position sensor (potentiometer).
        // NOTE: ***** Setting these values wrong can cause the arm and arm rotation hardware
        // to be damaged because the arm can be over-rotated and the gearbox on the motor makes it
        // very strong. *****

        //Change directions for shooter motors
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // Get the maximum voltage from the arm rotation potentiometer for calculating arm rotation
        // position in the [0, 1) range.
        //ARM_ROTATION_MAX_VOLTAGE = armRotationPositionSensor.getMaxVoltage();

        // Initialize settings for claw servo
        // Set a limited range for claw servo to prevent damage to the claw
        //lockServo.scaleRange(CLAW_SERVO_RANGE_MIN, CLAW_SERVO_RANGE_MAX);

        // Define IMU hardware instance variable
        //imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Set the IMU orientation on the robot
        //imu.initialize(
        //        new IMU.Parameters(
        //                new RevHubOrientationOnRobot(
        //                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        //                    RevHubOrientationOnRobot.UsbFacingDirection.UP
        //                )
        //        )
        //);

        // Define distance sensor hardware instance variables
        //frontDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "front_distance");
        //MAY REMOVE
        //rearDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "rear_distance");

        // Initialize the vision portal and the AprilTag processor(s)
        //if (vision)
        //initVision();
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init() {
        init(false);
    }

    /* ----- Low level motion methods for four-motor Mecanum drive train ----- */

    /**
     * Set Mecanum drivetrain motor powers directly.
     * Applies any defined correction values.
     */
    public void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {

        /*
         * NOTE: since we are using encoders on all wheels, what we are really doing here is
         * specifying a ratio (0.0 to 1.0) of the maximum RPM speed of the motor. Similar to
         * calling setVelocity() but without having to know the RPM ranges and values. The motor
         * controller code (DcMotorEx class) can then take care of changing voltage levels from the
         * battery and variations in friction in the motors for weight distribution of the robot to
         * provide more balanced speed control to the four wheels. The actual speed of the motor is
         * managed by the DcMotorEx class utilizing a built-in PID controller to attain the
         * calculated RPM. We may want to adjust the PID controller gain values via methods in the
         * DcMotorEx class to obtain stable operation.
         */

        // Send powers to the wheels and apply corrections.
        leftFrontDrive.setPower(leftFrontPower * OMNI_CORRECTION_LEFT_FRONT);
        rightFrontDrive.setPower(rightFrontPower * OMNI_CORRECTION_RIGHT_FRONT);
        leftBackDrive.setPower(leftBackPower * OMNI_CORRECTION_LEFT_BACK);
        rightBackDrive.setPower(rightBackPower * OMNI_CORRECTION_RIGHT_BACK);
    }

    /**
     * Drive robot according to robot-oriented axes of motion.
     * This method can be used by teleop OpModes directly to drive the robot, since the human on
     * the gamepad will be viewing and controlling the robot on the field with subtle adjustments
     * (thus "closed-loop" controller). It is also called by the higher-level motion routines in
     * the RobotHardware class for autonomous driving.
     *
     * @param x     "power" (relative speed) for axial movement (+ is forward)
     * @param y     power for lateral movement (strafe) (+ is left)
     * @param yaw   power for rotation (+ is counter-clockwise)
     * @param speed factor to scale the power values (0.0 to 1.0). Use MOTOR_SPEED_FACTOR_NORMAL, MOTOR_SPEED_FACTOR_DAVIS, or MOTOR_SPEED_FACTOR_PRECISE
     */
    public void move(double x, double y, double yaw, double speed) {

        // Calculate the powers for the four motors attached to the Mecanum wheels based on the
        // specified x, y, yaw powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0 but retain the balance between the four
        // wheels calculated above.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // adjust normalized powers by the speed factor and set motor powers
        setMotorPowers(
                leftFrontPower * speed,
                rightFrontPower * speed,
                leftBackPower * speed,
                rightBackPower * speed
        );
    }

    /**
     * Stop robot motion.
     */
    public void stop() {

        // Set all motor powers to zero
        setMotorPowers(0.0, 0.0, 0.0, 0.0);
    }

    /* ----- Methods for three-wheel odometry ----- */

    /**
     * Read odometry wheel encoders and update the current odometry counters and field position
     * of the robot.
     * This method should be called at the beginning of each loop in an autonomous opMode and in any
     * sub-loops in translation or rotation routines.
     */
    public void updateOdometry() {

        /*
         * NOTE: This code is adapted from the discussion of odometry in Game Manual 0 found here:
         * https://gm0.org/en/latest/docs/software/concepts/odometry.html. The code currently
         * implements linear approximations of deltas, i.e., assuming that the individual x and y
         * movements between updates occurred in straight lines. This may be fine for the mid-level
         * autonomous movement functions (forward, strafe, turn), but may build up errors over time
         * with movement functions or user driving. It would be more accurate if we use differential
         * equations (referred to as "Pose Exponentials" in the GM0 documentation) for computing the
         * deltas, i.e., assume that the movement between updates occurs in arcs. The differential
         * equation calculations converge to zero error faster than the linear approximations as the
         * frequency of odometry updates increases, i.e., loop times decrease.
         */

        // save current encoder values
        int oldRightCounter = lastRightEncoderPosition;
        int oldLeftCounter = lastLeftEncoderPosition;
        int oldAuxOdometryCounter = lastAuxEncoderPosition;

        // read new encoder values from odometry deadwheels and adjust for direction
        lastRightEncoderPosition = rightDeadwheelEncoder.getCurrentPosition() * DEADWHEEL_RIGHT_DIRECTION;
        lastLeftEncoderPosition = leftDeadwheelEncoder.getCurrentPosition() * DEADWHEEL_LEFT_DIRECTION;
        lastAuxEncoderPosition = auxDeadwheelEncoder.getCurrentPosition() * DEADWHEEL_AUX_DIRECTION;

        // calculate x, y, n1. d equals delta                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       and theta (heading) deltas (robot perspective) since last measurement
        int dl = lastLeftEncoderPosition - oldLeftCounter;
        int dr = lastRightEncoderPosition - oldRightCounter;
        int da = lastAuxEncoderPosition - oldAuxOdometryCounter;

        // Change in X is the just the average of the forward movement of the left and right deadwheels
        double dx = DEADWHEEL_MM_PER_TICK * (dl + dr) / 2.0;

        // Linear approximation of the change in heading (dtheta)
        double dtheta = DEADWHEEL_MM_PER_TICK * (dr - dl) / DEADWHEEL_TRACKWIDTH;

        // The linear approximation above becomes less accurate as (dr - dl) grows large. The
        // equations below provide a more accurate approximation that uses the law of cosines to
        // calculate the change in heading (dtheta):
        //      cos(theta) = a^2 + b^2 - c^2 / 2ab
        // where a = b = trackwidth / 2. However, the calculation utilizes several calls to the
        // Math library which may be computational intensive. As long as updateOdometry() is called
        // frequently enough, i.e., the change in odometry wheel ticks remains relatively small,
        // the linear approximation should be sufficient.
        //double c = DEADWHEEL_MM_PER_TICK * (dr - dl) / 2.0;
        //double dtheta = (c / Math.abs(c)) * Math.acos(1 - (2 * Math.pow(c, 2)) / Math.pow(DEADWHEEL_TRACKWIDTH, 2));

        // The change in Y is the forward movement of the aux deadwheel minus and ticks in the
        // deadwheel from change in heading
        double dy = DEADWHEEL_MM_PER_TICK * da - DEADWHEEL_FORWARD_OFFSET * dtheta;

        // update the x, y, and heading odometry counters
        xOdometryCounter += dx;
        yOdometryCounter += dy;
        headingOdometryCounter += dtheta;

        // update the current position in field coordinate system from the deltas
        // NOTE: disable this code for now since we are not using the field position and it may be
        // making the runtime too long.
        // NOTE: We may want to store the field position in a structure that is not as intensive to
        // update (i.e., one where a new object instance doesn't have to be created each time).
        //double theta = currentFieldPosition.getHeading(AngleUnit.RADIANS) + (dtheta / 2);
        //double newX = currentFieldPosition.getX(DistanceUnit.MM) + dx * Math.cos(theta) - dy * Math.sin(theta);
        //double newY = currentFieldPosition.getY(DistanceUnit.MM) + dx * Math.sin(theta) + dy * Math.cos(theta);
        //double newHeading = (currentFieldPosition.getHeading(AngleUnit.RADIANS) + dtheta) % (2.0 * Math.PI); // normalize to [0, 2pi)
        //if(newHeading < 0) {
        //    newHeading += 2.0 * Math.PI;
        //}
        //currentFieldPosition = new Pose2D(DistanceUnit.MM, newX, newY, AngleUnit.RADIANS, newHeading);
    }

    /**
     * Reset the x, y, and heading odometry counters to zero.
     * This method should be called at the beginning of simple move and rotate commands to ensure
     * that translation and rotation are relevant to robot's starting position.
     */
    public void resetOdometryCounters() {

        // reset the odometry counters to zero
        xOdometryCounter = 0.0;
        yOdometryCounter = 0.0;
        headingOdometryCounter = 0.0;
    }

    /**
     * Return axial (x) odometry counter in MM units.
     * This method is primarily for retrieval of the odometry counters by the OpMode for display
     * in telemetry during testing.
     */
    public double getOdometryX() {
        return xOdometryCounter;
    }

    /**
     * Return lateral (y) odometry counter in MM units.
     * This method is primarily for retrieval of the odometry counters by the OpMode for display
     * in telemetry during testing.
     */
    public double getOdometryY() {
        return yOdometryCounter;
    }

    /**
     * Return heading odometry counter in radians.
     * This method is primarily for retrieval of the odometry counters by the OpMode for display
     * in telemetry during testing.
     */
    public double getOdometryHeading() {
        return headingOdometryCounter;
    }

    /* ----- Mid-level motion methods for autonomous motion ----- */

    /**
     * Drive forward (reverse) while maintaining current heading and limiting sideways drift.
     * This method should only be called from a LinerOpMode and implements its own loop to cover the
     * robot's motion to the specified position.
     *
     * @param distance Distance (MM) to move: + is forward, - is reverse
     * @param speed    Speed factor to apply (should use defined constants)
     */
    public void forward(double distance, double speed) {

        // Proportional controllers for x, y, and yaw
        PController xController = new PController(distance, X_POSITION_TOLERANCE, X_CONTROLLER_DEADBAND, X_CONTROLLER_KP);
        PController yController = new PController(0.0, Y_POSITION_TOLERANCE, Y_CONTROLLER_DEADBAND, Y_CONTROLLER_KP);
        PController yawController = new PController(0.0, HEADING_TOLERANCE, YAW_CONTROLLER_DEADBAND, YAW_CONTROLLER_KP);

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        // NOTE: opModeIsActive() calls idle() internally, so we don't need to call idle()
        // in the loop
        while (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive()) {

            // update odometry counters
            updateOdometry();

            // If we have reached the desired position, break out of the loop
            if (xController.isWithinTolerance(xOdometryCounter))
                break;

            // Calculate the control output for each of the three controllers
            double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            //double yPower = 0.0;
            double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);
            //double yawPower = 0.0;

            // Move the robot based on the calculated powers
            move(xPower, yPower, yawPower, speed);
        }

        // stop movement of the robot
        stop();
    }

    /**
     * Strafe left (right) while maintaining current heading and limiting forward/backward drift.
     * This method should be called from a LinerOpMode and implements its own loop to cover the
     * robot's motion to the specified position.
     *
     * @param distance Distance (MM) to move: + is left, - is right
     * @param speed    Speed factor to apply (should use defined constants)
     */
    public void strafe(double distance, double speed) {

        // Proportional controllers for x, y, and yaw
        PController xController = new PController(0.0, X_POSITION_TOLERANCE, X_CONTROLLER_DEADBAND, X_CONTROLLER_KP);
        PController yController = new PController(distance, Y_POSITION_TOLERANCE, Y_CONTROLLER_DEADBAND, Y_CONTROLLER_KP);
        PController yawController = new PController(0.0, HEADING_TOLERANCE, YAW_CONTROLLER_DEADBAND, YAW_CONTROLLER_KP);

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        // NOTE: opModeIsActive() calls idle() internally, so we don't need to call idle()
        // in the loop
        while (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive()) {

            // Update the odometry counters
            updateOdometry();

            // If we have reached the desired position, break out of the loop
            if (yController.isWithinTolerance(yOdometryCounter))
                break;

            // Calculate the control output for each of the three controllers
            double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            //double xPower = 0.0;
            double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);
            //double yawPower = 0.0;

            // Move the robot based on the calculated powers
            move(xPower, yPower, yawPower, speed);
        }

        myOpMode.telemetry.addData("Y Odometry:", yOdometryCounter);
        myOpMode.telemetry.update();

        // stop the robot
        stop();
    }

    /**
     * Turn a relative angle.
     * This method should be called from a LinerOpMode and implements its own loop to cover the
     * robot's motion to the specified position.
     *
     * @param angle Angle to rotate in Radians: + is counter-clockwise, - is clockwise
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void turn(double angle, double speed) {

        // Proportional controller heading
        // NOTE: Maintaining proportional controllers for x and y at zero to prevent drift doesn't
        // seem to work very well due to the linear nature of the odometry calculations and/or the
        // lack of calibration of DEADWHEEL_FORWARD_OFFSET parameter. For now, we just use a simple
        // P-controller for heading.
        PController yawController = new PController(angle, HEADING_TOLERANCE, YAW_CONTROLLER_DEADBAND, YAW_CONTROLLER_KP);

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        // NOTE: opModeIsActive() calls idle() internally, so we don't need to call idle()
        // in the loop
        while (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive()) {

            // Update the odometry counters
            updateOdometry();

            // If we have reached the desired position, break out of the loop
            if (yawController.isWithinTolerance(headingOdometryCounter))
                break;

            // Calculate the control output for each of the three controllers
            double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);

            // Move the robot based on the calculated powers
            // NOTE: We reduced the yaw power by 60% to make the robot turn more slowly and accurately
            move(0, 0, yawPower, speed * 0.6);
        }

        // stop the robot
        stop();
    }

    /* ------ Shooter Methods ----- */
    public void shoot() {
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //Spin the wheel
        launcherMotor.setPower(0.85);
        //Wait for 1/2 a second
        sleep(700);
        //Stop spinning the wheel
        launcherMotor.setPower(0);
    }
    public void shootOn(boolean far) {
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        if (far)
        {
            launcherMotor.setPower(.78);
        }
        else
        {
            launcherMotor.setPower(0.78);


        }
    }
    public void shootOff() {
        launcherMotor.setPower(0);
    }
    /*------ Kebob Methods ------ */

    public void kebobOff() {
        kebobMotor.setPower(0);
    }


    public void reverseLauncher() {
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Spin the wheel
        launcherMotor.setPower(0.5);
        //Wait for 1/2 a second
        sleep(700);
        //Stop spinning the wheel
        launcherMotor.setPower(0);
    }
    public void reverseKebob(double power) {
        kebobMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        kebobMotor.setPower(power);
    }
    public void forwardKebob(double power) {
        kebobMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        kebobMotor.setPower(power - .1);
    }

    /* ------- Odometry Calculation ----- */
    public void strafeTest()
    {
        forward(100, MOTOR_SPEED_FACTOR_PRECISE);
        strafe(100, MOTOR_SPEED_FACTOR_AUTONOMOUS);
    }

    /* ----- Vision processing methods ----- */

    // Initialize VisionPortal and AprilTagProcessor objects
    public void initVision()
    {

        // Create and build an AprilTag processor for each camera.
        aprilTagCam1 = new AprilTagProcessor.Builder()

            // un-comment and edit the following default settings as needed
                .setDrawAxes(true)
            //.setDrawCubeProjection(false)
            .setDrawTagOutline(false)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)
            .setCameraPose(cam1Position, cam1Orientation)
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.
            .build();

        aprilTagCam2 = new AprilTagProcessor.Builder()

                // un-comment and edit the following default settings as needed
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)
                .setCameraPose(cam2Position, cam2Orientation)
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Adjust image decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagCam1.setDecimation(3);
        aprilTagCam2.setDecimation(3);

        // setup webcam references for each camera
        webcam1 = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName switchableCamera = ClassFactory.getInstance()
            .getCameraManager().nameForSwitchableCamera(webcam1);

        // Create the vision portal by using a builder and setup for multiple webcams
        visionPortal = new VisionPortal.Builder()
            .setCamera(switchableCamera)

            // un-comment and edit the following default settings as needed
            .enableLiveView(true) // Enable the RC preview (LiveView) - set "false" to omit camera monitoring
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //.setAutoStopLiveView(false) //Choose whether or not LiveView stops if no processors are enabled.
            .setCameraResolution(new Size(1920, 1080)) // camera resolution (for all cameras ???)
            .addProcessor(aprilTagCam1)
            .addProcessor(aprilTagCam2)
            .build();

        // Disable the aprilTag processor(s) until a camera is selected
        visionPortal.setProcessorEnabled(aprilTagCam1, false);
        visionPortal.setProcessorEnabled(aprilTagCam2, false);
    }

    /**
     * Turn on a specific camera for AprilTag detection. Activate the corresponding AprilTag
     * detector as well.
     * @param camera the number of the camera to use - 0 deactivates all cameras
     */
    public void switchCamera(int camera) {

        // no need for anything fancy, just select the right camera and activate
        // the corresponding AprilTag processor (and deactivate all others)
        if (camera == 1) {
            visionPortal.setActiveCamera(webcam1);
            visionPortal.setProcessorEnabled(aprilTagCam1, true);
            visionPortal.setProcessorEnabled(aprilTagCam2, false);
        }
        // if we have a second camera, activate it and its processor
        else if (camera == 2) {
            visionPortal.setActiveCamera(webcam2);
            visionPortal.setProcessorEnabled(aprilTagCam2, true);
            visionPortal.setProcessorEnabled(aprilTagCam1, false);
        }
        // if no camera specified, disable the AprilTag processor(s)
        else if (camera == 0) {
            visionPortal.setProcessorEnabled(aprilTagCam1, false);
            visionPortal.setProcessorEnabled(aprilTagCam2, false);
        }
    }

    /**
     * Get AprilTag detections from the currently active camera.
     */
    public ArrayList<AprilTagDetection> getAprilTags() {
        if (visionPortal.getProcessorEnabled(aprilTagCam1))
            return aprilTagCam1.getDetections();
        else if (visionPortal.getProcessorEnabled(aprilTagCam2))
            return aprilTagCam2.getDetections();
        else
            return new ArrayList<>();
    }

    /**
     * Enable video streaming for AprilTag detection
     */
    public void enableVision() {
        visionPortal.resumeStreaming();
    }

    /**
     * Disable video streaming for AprilTag detection
     * Teleop OpModes may want to call this method to save hardware resources in teleop if
     * no AprilTag processing is needed.
     */
    public void disableVision() {
        visionPortal.stopStreaming();
    }

    /**
     * Returns the position and orientation (pose) of the specified AprilTage relative to the
     * specified camera. This can be used in autonomous OpModes to correct the robot's current
     * position based on the anticipated, correct pose of the AprilTag from where the robot should
     * be.
     * This method should only be called from a LinerOpMode and may delay for some period before
     * returning the Pose3D object. If specified tag cannot be detected, returns null.
     * //@param camera camera to use for AprilTag detection (int 1, 2, etc.)
     * //@param tagID tag ID to use to get position (int from competition library)
     * //@return Pose3D object with the position and orientation of the robot relative to the tag
     * //@see org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
     */
    /* public AprilTagPoseFtc getAprilTagPose(int camera, int tagID) {

        // Maximum number of times the specified tag was not detected before breaking out of the loop
        final int MAX_NO_DETECTION_COUNT = 5;

        // current pose of robot
        AprilTagPoseFtc tagPose = null;

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // If vision was not initialized or camera(s) are not enabled, then return
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return null;

        // Switch to the specified camera
        switchCamera(camera);

        // Counter for the number of times the specified tag was not detected
        int notDetectedCount = 0;

        // Loop until the the tag is detected
        // NOTE: opModeIsActive() calls idle() internally, so we don't need to call idle()
        // in the loop
        // NOTE: Does this need to be done in two loops: one for traversal and one for rotation?
        while (notDetectedCount <= MAX_NO_DETECTION_COUNT && (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive())) {

            // get the latest AprilTag detections
            ArrayList<AprilTagDetection> aprilTags = getAprilTags();

            // Find specified tag in the current detections (or set the first one found if no tag
            // ID specified)
            AprilTagDetection target = null;
            for (AprilTagDetection tag : aprilTags) {
                if (tag.id == tagID || tagID == 0) {
                    target = tag;
                    break;  // don't look any further
                }
            }

            // If the specified tag was detected, return the pose of the tag
            if (target != null) {

                // retrieve the current field position of the robot and exit the loop
                tagPose = aprilTags.get(0).ftcPose;
                break;
            }

            // otherwise, if the specified tag was not detected, increment the counter
            else
                notDetectedCount++;

            // Wait some time for robot to move and new AprilTag detections to be acquired
            if(isLinearOpMode)
                ((LinearOpMode) myOpMode).sleep(100);
        }

        // Turn off AprilTag detection
        switchCamera(0);

        // return the detected robot position (if any)
        return tagPose;
    }

    /**
     * Returns the longitudinal (camera's Y+ axis) distance from the specified camera
     * to the specified AprilTag. Note this is not a direct line distance (range) to the tag, but
     * the distance along a line straight out of the camera lens and is always positive.
     * This method should only be called from a LinerOpMode and may delay for some period before
     * returning the distance. If specified tag cannot be detected, returns -1.
     * @param camera camera to use for AprilTag detection (int 1, 2, etc.)
     * @param tagID tag ID to use to get position (int from competition library)
     * @return distance in MM or 0 if tag is not detected
     */
    /*public double getLongitudinalDistanceToAprilTag(int camera, int tagID) {

        // Maximum number of times the specified tag was not detected before breaking out of the loop
        final int MAX_NO_DETECTION_COUNT = 5;

        // return value for the distance to the tag (0 for not detected)
        double dist = 0.0;

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // If vision was not initialized or camera(s) are not enabled, then return
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return dist;

        // Switch to the specified camera
        switchCamera(camera);

        // Counter for the number of times the specified tag was not detected
        int notDetectedCount = 0;

        // Loop until the the tag is detected
        // NOTE: opModeIsActive() calls idle() internally, so we don't need to call idle()
        // in the loop
        // NOTE: Does this need to be done in two loops: one for traversal and one for rotation?
        while (notDetectedCount <= MAX_NO_DETECTION_COUNT && (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive())) {

            // get the latest AprilTag detections
            ArrayList<AprilTagDetection> aprilTags = getAprilTags();

            // Find specified tag in the current detections (or set the first one found if no tag
            // ID specified)
            AprilTagDetection target = null;
            for (AprilTagDetection tag : aprilTags) {
                if (tag.id == tagID || tagID == 0) {
                    target = tag;
                    break;  // don't look any further
                }
            }

            // If the specified tag was detected, calculate the distance to the tag
            if (target != null) {

                // get the Y axis distance to the pose of the AprilTag and exit the loop
                dist = aprilTags.get(0).ftcPose.y;
                break;
            }

            // otherwise, if the specified tag was not detected, increment the counter
            else
                notDetectedCount++;

            // Wait some time for robot to move and new AprilTag detections to be acquired
            if(isLinearOpMode)
                ((LinearOpMode) myOpMode).sleep(100);
        }

        // Turn off AprilTag detection
        switchCamera(0);

        // return the distance along detected robot position (if any)
        return dist;
    }

    /**
     * Move robot to specified field coordinate position (X, Y) and heading in MM and radians based
     * on the specified AprilTag detected by specified camera.
     * This method should only be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified position.
     * @param x x-coordinate of center of robot in field coordinates (MM)
     * @param y y-coordinate of center of robot in field coordinates (MM)
     * @param heading current angle of robot relative to positive x-axis in field coordinates (rad)
     * @param camera camera to use for AprilTag detection (int 1, 2, etc.)
     * @param tagID tag ID to use to get position (int from competition library)
     * @param speed Speed factor to apply (should use defined constants)
     */
    /*public void moveToPositionUsingAprilTag(double x, double y, double heading, int camera, int tagID, double speed) {

        // Maximum number of times the specified tag was not detected before breaking out of the loop
        final int MAX_NO_DETECTION_COUNT = 5;

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // The first version of this function moved and oriented the robot in all three "axes" (X,
        // Y, and heading) at the same time. While it worked, it was wonky in the translation with
        // overshoot and correction. This may be due to the fact that the transform (rotation) of
        // coordinates from the field coordinate plane to robot-oriented plane used is inherently
        // linear and calculating x,y translation along with rotation just doesn't work out.
        // This version uses two passes through the camera loop: 1) yaw (rotate) to correct the
        // heading, then 2) move the robot to the correct x, y position while maintaining heading.
        // This will (hopefully) reduce the wonky movement.
        boolean firstPass = true;

        // If vision was not initialized or camera(s) are not enabled, then return
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return;

        // Switch to the specified camera
        switchCamera(camera);

        // Counter for the number of times the specified tag was not detected
        int notDetectedCount = 0;

        // Loop until the robot has reached the desired position
        // NOTE: opModeIsActive() calls idle() internally, so we don't need to call idle()
        // in the loop
        // NOTE: Does this need to be done in two loops: one for traversal and one for rotation?
        while (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive()) {

            // get the latest AprilTag detections
            ArrayList<AprilTagDetection> aprilTags = getAprilTags();

            // Find specified tag in the current detections (or set the first one found if no tag
            // ID specified)
            AprilTagDetection target = null;
            for (AprilTagDetection tag : aprilTags) {
                if (tag.id == tagID || tagID == 0) {
                    target = tag;
                    break;  // don't look any further
                }
            }

            // If the specified tag was detected, calculate robot movement
            if (target != null) {

                // reset not detected count
                notDetectedCount = 0;

                // retrieve the current field position of the robot
                Pose3D currentPos = aprilTags.get(0).robotPose;

                // calculate error in the x, y, and heading from the current robot pose and the
                // specified target position
                // NOTE: The deltas (errors) for x and y in the robot's axes are calculated from the
                // delta x and delta y in the field coordinate system by applying a rotation transform
                // using the robot's current heading (in field coordinates):
                double deltaX = x - currentPos.getPosition().x;
                double deltaY = y - currentPos.getPosition().y;
                double theta = currentPos.getOrientation().getYaw(AngleUnit.RADIANS);
                double errorX = deltaX * Math.cos(theta) + deltaY * Math.sin(theta);
                double errorY = -deltaX * Math.sin(theta) + deltaY * Math.cos(theta);
                double errorH = heading - theta;

                // show the current robot position, target position, and calculated errors in telemetry
                // NOTE: This is just for debugging purposes and can be removed in production code
                myOpMode.telemetry.addData(
                        "Robot Position ",
                        "X: %6.1f, Y: %6.1f, H: %1.3f",
                        currentPos.getPosition().x,
                        currentPos.getPosition().y,
                        currentPos.getOrientation().getYaw(AngleUnit.RADIANS));
                myOpMode.telemetry.addData(
                        "Target Position",
                        "X: %6.1f, Y: %6.1f, H: %1.3f",
                        x,
                        y,
                        heading);
                myOpMode.telemetry.addData(
                        "Computed Error",
                        "X: %6.1f, Y: %6.1f, H: %1.3f",
                        errorX,
                        errorY,
                        errorH);
                myOpMode.telemetry.update();

                // If we have reached the desired position, break out of the loop
                if (Math.abs(errorX) < X_POSITION_TOLERANCE &&
                        Math.abs(errorY) < Y_POSITION_TOLERANCE &&
                        Math.abs(errorH) < HEADING_TOLERANCE)
                    break;
                // otherwise, if the we are in the first (heading only) pass and the heading is
                // within tolerance, then proceed to the second (x,y movement) pass.
                else if (firstPass && Math.abs(errorH) < HEADING_TOLERANCE)
                    firstPass = false;

                // Calculate "power" for each of the three motor axes using proportional gains
                // NOTE: for the first (heading only) pass, just set the x and y power values to
                // zero.
                double xPower = firstPass ? 0 : clip(errorX * X_CONTROLLER_KP, -1.0, 1.0);
                double yPower = firstPass ? 0 : clip(errorY * Y_CONTROLLER_KP, -1.0, 1.0);
                double yawPower = clip(errorH * YAW_CONTROLLER_KP, -1.0, 1.0);

                // Move the robot based on the calculated powers
                move(xPower, yPower, yawPower, speed);
            }
            else {
                // If the specified tag was not detected, increment the counter
                notDetectedCount++;

                // If not detected counts exceeds maximum number of times, break out
                // of the loop
                if (notDetectedCount > MAX_NO_DETECTION_COUNT)
                    break;
            }

            // Wait some time for robot to move and new AprilTag detections to be acquired
            if(isLinearOpMode)
                ((LinearOpMode) myOpMode).sleep(100);
        }

        // stop any robot movement
        stop();

        // Turn off AprilTag detection
        switchCamera(0);
    }
}

/**
 * Proportional Controller class for computing motor power during autonomous motion.
 * NOTE: Could have used the PID controller class passing 0s for Ki and Kd, but adding a bunch of
 * if statements to remove the unnecessary calculations (for performance sake) made the code hard
 * to read.
 */
    class PController {

        // properties to store values needed for repeated calculations
        private final double target; // target position initially provided
        private final double tolerance; // tolerance range for ending controller function
        private final double deadband; // deadband range for returning zero power
        private final double Kp; // proportional gain

        // Constructor to set the controller parameters
        public PController(double target, double tolerance, double deadband, double Kp) {
            this.target = target;
            this.tolerance = tolerance;
            this.deadband = deadband;
            this.Kp = Kp;
        }

        // Method to determine if error is within tolerance range
        public boolean isWithinTolerance(double currentPosition) {
            return Math.abs(target - currentPosition) < tolerance;
        }

        // Method to calculate the control output based on the current position
        public double calculate(double currentPosition) {

            // Calculate the error
            double error = target - currentPosition;

            // Check if the error is within the deadband range
            if (Math.abs(error) < deadband)
                return 0.0;
            else
                // Calculate the control output
                return Kp * error;
        }
    }

    /**
     * PID Controller class for computing motor power during autonomous motion.
     */
    @SuppressWarnings("unused")
    class PIDController {

        // properties to store values needed for repeated calculations
        private final double target; // target position initially provided
        private final double tolerance; // tolerance range for ending controller function
        private final double deadband; // deadband range for returning zero power
        private final double Kp; // proportional gain
        private final double Ki;
        private final double Kd;

        // tracking values
        private double integralSum = 0.0;
        private double lastError = 0.0;
        private int lastTime = 0;

        // Constructor to set the PID controller parameters with all (PID) gain values
        @SuppressWarnings("unused")
        public PIDController(double target, double tolerance, double deadband, double Kp, double Ki, double Kd) {
            this.target = target;
            this.deadband = deadband;
            this.tolerance = tolerance;
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        // Method to determine if error is within tolerance range
        @SuppressWarnings("unused")
        public boolean isWithinTolerance(double currentPosition) {
            return Math.abs(target - currentPosition) < tolerance;
        }

        // Method to calculate the control output based on the current position
        @SuppressWarnings("unused")
        public double calculate(double currentPosition) {

            // Calculate the error
            double error = target - currentPosition;

            // Check if the error is within the deadband range
            if (Math.abs(error) < deadband)
                return 0.0;
            else {

                // Get elapsed time (secs) since last calculation
                int currentTime = (int) System.currentTimeMillis() / 1000;
                int deltaTime = currentTime - lastTime;
                lastTime = currentTime;

                // Update the integral sum
                integralSum += error * deltaTime;

                // Calculate the derivative term
                double derivative = (error - lastError) / deltaTime; // rate of change of the error

                // update the last error value
                lastError = error;

                // Calculate the control output
                return (Kp * error) + (Ki * integralSum) + (Kd * derivative);


            }
        }
    }
}