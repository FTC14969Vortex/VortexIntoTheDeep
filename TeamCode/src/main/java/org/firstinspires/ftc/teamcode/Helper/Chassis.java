package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.Helper.ProportionalControl;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class Chassis {

    /*
   Inches per count calculation: diameter is 32mm (from goBuilda website) so the circumference is  pi*diameter = 100.53096491487338mm.
   100.53096491487338mm = 0.003952755905511811inches.
   The encoder is 2000 counts per revolution. So the inches per count is 0.003952755905511811/2000 = 0.0019763779527559055.
    */
    private final double ODOM_INCHES_PER_COUNT   = 0.001978956;   //  GoBilda Odometry Pod (1/226.8)

    private final boolean INVERT_DRIVE_ODOMETRY  = true;       //  When driving FORWARD, the odometry value MUST increase.  If it does not, flip the value of this constant.
    private final boolean INVERT_STRAFE_ODOMETRY = false;       //  When strafing to the LEFT, the odometry value MUST increase.  If it does not, flip the value of this constant.

    private static final double DRIVE_GAIN          = 0.03;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.6;     // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.03;    // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.018;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);


    //Drivetrain Motor
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;

    public DcMotor driveEncoder;       //  the Axial (front/back) Odometry Module (may overlap with motor, or may not)
    public DcMotor strafeEncoder;      //  the Lateral (left/right) Odometry Module (may overlap with motor, or may not)

    //IMU
    public  IMU imu;

    public YawPitchRollAngles angles;

    private LinearOpMode myOpMode;

    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private double rawDriveOdometer    = 0; // Unmodified axial odometer count
    private double driveOdometerOffset = -13; // Used to offset axial odometer
    private double rawStrafeOdometer   = 0; // Unmodified lateral odometer count
    private double strafeOdometerOffset= -139.7; // Used to offset lateral odometer
    private double rawHeading       = 0; // Unmodified heading (degrees)
    private double headingOffset    = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry     = false;

    //Constructor
    public Chassis(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void init(boolean showTelemetry) throws InterruptedException {

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        FLMotor = setupDriveMotor("FLMotor", DcMotor.Direction.REVERSE);
        FRMotor = setupDriveMotor("FRMotor", DcMotor.Direction.FORWARD);
        BLMotor = setupDriveMotor( "BLMotor", DcMotor.Direction.REVERSE);
        BRMotor = setupDriveMotor( "BRMotor",DcMotor.Direction.FORWARD);

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        //  Connect to the encoder channels using the name of that channel.
        driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "OdoX");
        strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "OdoY");

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // zero out all the odometry readings.
        resetOdometry();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {

        rawDriveOdometer = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        rawStrafeOdometer = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);
        driveDistance = (rawDriveOdometer - driveOdometerOffset) * ODOM_INCHES_PER_COUNT;
        strafeDistance = (rawStrafeOdometer - strafeOdometerOffset) * ODOM_INCHES_PER_COUNT;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        heading     = rawHeading - headingOffset;
        turnRate    = angularVelocity.zRotationRate;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Odom X", rawDriveOdometer - driveOdometerOffset);
            myOpMode.telemetry.addData("Odom Y", rawStrafeOdometer - strafeOdometerOffset);
            myOpMode.telemetry.addData("Dist X", driveDistance);
            myOpMode.telemetry.addData("Dist Y", strafeDistance);
            myOpMode.telemetry.addData("Heading", heading);
        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(distanceInches, power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(distanceInches, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {

        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    public boolean isRobotStable(){
        angles = imu.getRobotYawPitchRollAngles();

        if(Math.abs(angles.getPitch(AngleUnit.DEGREES)) >= 5 || Math.abs(angles.getRoll(AngleUnit.DEGREES)) >= 5) {
            return false;
        } else {
            return true;
        }
    }

    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double FLPower = drive - strafe - yaw;
        double FRPower = drive + strafe + yaw;
        double BLPower = drive + strafe - yaw;
        double BRPower = drive - strafe + yaw;

        double max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
        max = Math.max(max, Math.abs(BLPower));
        max = Math.max(max, Math.abs(BRPower));

        //normalize the motor values
        if (max > 1.0)  {
            FLPower /= max;
            FRPower /= max;
            BLPower /= max;
            BRPower /= max;
        }

        //send power to the motors
        FLMotor.setPower(FLPower);
        FRMotor.setPower(FRPower);
        BLMotor.setPower(BLPower);
        BRMotor.setPower(BRPower);
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Set odometry counts and distances to zero.
     */
    public void resetOdometry() {
        readSensors();
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveController.reset(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeController.reset(0);
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    public void resetAll () {
        resetOdometry();
        resetHeading();
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }
}

