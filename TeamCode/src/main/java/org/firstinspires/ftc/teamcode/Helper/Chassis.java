package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Arrays;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class Chassis {

    // global location.
    public int robotX = 0;
    public int robotY = 0;

    public int[] Location = {robotX,robotY};


    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    //IMU
    public  IMU imu;

    public YawPitchRollAngles angles;

    private ElapsedTime runtime = new ElapsedTime();

    int timeout_ms;




    //Drivetrain Motor
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;

    //IMU
    //How many times the encoder counts a tick per revolution of the motor.
    static final double COUNTS_PER_MOTOR_REV_Hex= 538; // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    //Gear ratio of the motor to the wheel. 1:1 would mean that 1 turn of the motor is one turn of the wheel, 2:1 would mean two turns of the motor is one turn of the wheel, and so on.
    static final double DRIVE_GEAR_REDUCTION= 1; // This is < 1.0 if geared UP

    //Diameter of the wheel in inches
    static final double WHEEL_DIAMETER_IN = 3.93701; // For figuring circumference

    //How many times the encoder counts a tick per inch moved. (Ticks per rev * Gear ration) / perimeter
    static final double COUNTS_PER_IN = (COUNTS_PER_MOTOR_REV_Hex * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_IN * 3.1415);

    HardwareMap hwMap = null;



    //private static LinearOpmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;

        //Init motors and servos
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");

        odo = hwMap.get(GoBildaPinpointDriver.class,"odo");
        initOdometry();



        //Setting the direction
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set behavior when zero power is applied.
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        imu = hwMap.get(IMU.class, "imu");

        // Set the mounting orientation of the IMU sensor.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        // Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public boolean isRobotStable(){
        angles = imu.getRobotYawPitchRollAngles();

        if(Math.abs(angles.getPitch(AngleUnit.DEGREES)) >= 5 || Math.abs(angles.getRoll(AngleUnit.DEGREES)) >= 5) {
            return false;
        } else {
            return true;
        }
    }

    public void initOdometry() {
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-13, -139.7);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.resetPosAndIMU();
    }

    //Return the position from the odometry wheels as a string (Can be used to print into telemetry)
    public String getOdoPos() {
        odo.update();
         /*
            gets the current Position (x & y in inches, and heading in degrees) of the robot, and prints it.
             */
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        return data;
    }

    //Return the velocity from the odometry wheels as a string (Can be used to print into telemetry)
    public String getOdoVel() {
        odo.update();
        /*
            gets the current Velocity (x & y in in/sec and heading in degrees/sec) and prints it.
             */
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
        return velocity;
    }

    public void Drive(double speed, float distance) {

        runtime.reset();
        timeout_ms = 10000;

        robotY += distance;
        Location[1] = robotY;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = FLMotor.getCurrentPosition();
        int FRPos = FRMotor.getCurrentPosition();
        int BLPos = BLMotor.getCurrentPosition();
        int BRPos = BRMotor.getCurrentPosition();

        targetFR = FRPos + (int) (distance * COUNTS_PER_IN);
        targetBR = BRPos + (int) (distance * COUNTS_PER_IN);
        targetFL = FLPos + (int) (distance * COUNTS_PER_IN);
        targetBL = BLPos + (int) (distance * COUNTS_PER_IN);

        //Set motor targets
        FLMotor.setTargetPosition(targetFL);
        BLMotor.setTargetPosition(targetBL);
        FRMotor.setTargetPosition(targetFR);
        BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (FLMotor.isBusy() && FRMotor.isBusy())) {
        }
        this.stopDriveMotors();
    }

    public void DriveWithOdo(double speed, double distance) {

    }

    public void Strafe(double speed, double distance) {

        robotX += distance;

        runtime.reset();
        timeout_ms = 10000;

        robotX += distance;
        Location[0] = robotX;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = FLMotor.getCurrentPosition();
        int FRPos = FRMotor.getCurrentPosition();
        int BLPos = BLMotor.getCurrentPosition();
        int BRPos = BRMotor.getCurrentPosition();

        targetFR = FRPos - (int) (distance * COUNTS_PER_IN);
        targetBR = BRPos + (int) (distance * COUNTS_PER_IN);
        targetFL = FLPos + (int) (distance * COUNTS_PER_IN);
        targetBL = BLPos - (int) (distance * COUNTS_PER_IN);



        //Set motor targets
        FLMotor.setTargetPosition(targetFL);
        BLMotor.setTargetPosition(targetBL);
        FRMotor.setTargetPosition(targetFR);
        BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (FLMotor.isBusy() && FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

    public void stopDriveMotors(){
        // Stop all motion;
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }

    public void DriveToPosition(double Speed, int posX, int posY, boolean forwardFirst) {
        if(forwardFirst) {
            this.Drive(Speed, -posY);
            this.Strafe(Speed, -posX);
        }
        else{
            this.Strafe(Speed, -posX);
            this.Drive(Speed, -posY);
        };
        System.out.println(Arrays.toString(Location));
    }
    public float modAngle(float angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }
    public void turnRobotToAngle(float endAngle) {
        YawPitchRollAngles angle;
        angle = imu.getRobotYawPitchRollAngles();

        float angleStart = modAngle((float) angle.getYaw(AngleUnit.DEGREES));
        float angleEnd = modAngle(endAngle);
        float angleCurrent = angleStart;
        float direction = 0;

        if (modAngle((angleEnd - angleCurrent)) >= 180) {
            //Go Clockwise
            direction = -1;
        } else if (modAngle((angleEnd - angleCurrent)) <= 180) {
            //Go Counter Clockwise
            direction = 1;
        }

        double pwr = -0.6;


        while (Math.abs(angleCurrent - angleEnd) > 2) {
            FLMotor.setPower(-pwr * direction);
            FRMotor.setPower(pwr * direction);
            BLMotor.setPower(-pwr * direction);
            BRMotor.setPower(pwr * direction);
            angle = imu.getRobotYawPitchRollAngles();
            angleCurrent = modAngle((float) angle.getYaw(AngleUnit.DEGREES));
        }
        stopDriveMotors();


    }
    public void autoTurn(float turnAngle, float turnOffset){
        float desc_start = 10;
        double acc = 1;
        double turnDirection = Math.signum(turnAngle);
        double startAngle;
        double currentAngle;
        double alreadyTurned = 0;
        double turnSpeed = 0.3;

        turnAngle = Math.abs(turnAngle) - turnOffset;


        this.FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (alreadyTurned< turnAngle){

            FLMotor.setPower(- turnSpeed * turnDirection * acc);
            FRMotor.setPower(turnSpeed * turnDirection* acc);
            BLMotor.setPower(-turnSpeed * turnDirection * acc);
            BRMotor.setPower(turnSpeed * turnDirection * acc);
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            alreadyTurned = Math.abs(currentAngle - startAngle);

            if (alreadyTurned>180){
                alreadyTurned = 360 - alreadyTurned;
            }
        }
        stopDriveMotors();

    }
}

