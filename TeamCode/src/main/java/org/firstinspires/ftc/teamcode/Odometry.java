package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto_Odometry_Wheels_Mecanum", group = "Auto")
public class AutoOdometryWheelsMecanum extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // Odometry wheels (encoders only, no motor power)
    private DcMotor leftOdo, rightOdo;

    // IMU sensor
    private BNO055IMU imu;

    // Odometry constants (example values, adjust for your hardware)
    private static final double ODO_TICKS_PER_REV = 8192;  // Example for REV Through-Bore Encoder
    private static final double ODO_WHEEL_DIAMETER_INCHES = 2.0; // Diameter of odometry wheels
    private static final double TICKS_PER_INCH = ODO_TICKS_PER_REV / (Math.PI * ODO_WHEEL_DIAMETER_INCHES);

    // Robot position variables in inches
    private double robotX = 0;
    private double robotY = 0;
    private double lastLeftOdoTicks = 0;
    private double lastRightOdoTicks = 0;

    // IMU heading in degrees
    private double robotHeading = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        leftOdo = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        // Reverse drive motors if needed
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Odometry wheels run without powering motors
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Reset odometry tracking
            lastLeftOdoTicks = leftOdo.getCurrentPosition();
            lastRightOdoTicks = rightOdo.getCurrentPosition();

            // Move in a triangle path using odometry tracking
            goToPosition(24, 0, 0.5, 10);
            goToPosition(24, 24, 0.5, 10);
            goToPosition(0, 0, 0.5, 10);

            stopMotors();
        }
    }

    // Update robot position using odometry wheels and IMU heading
    private void updatePosition() {
        double currentLeftTicks = leftOdo.getCurrentPosition();
        double currentRightTicks = rightOdo.getCurrentPosition();

        double deltaLeft = (currentLeftTicks - lastLeftOdoTicks) / TICKS_PER_INCH;
        double deltaRight = (currentRightTicks - lastRightOdoTicks) / TICKS_PER_INCH;

        lastLeftOdoTicks = currentLeftTicks;
        lastRightOdoTicks = currentRightTicks;

        // Average forward movement
        double deltaForward = (deltaLeft + deltaRight) / 2;

        // Get heading in radians
        robotHeading = imu.getAngularOrientation().firstAngle;

        // Calculate change in X and Y based on heading
        double deltaX = deltaForward * Math.cos(robotHeading);
        double deltaY = deltaForward * Math.sin(robotHeading);

        robotX += deltaX;
        robotY += deltaY;
    }

    // Drive to a target X,Y position (in inches), maxPower and timeoutSeconds for safety
    private void goToPosition(double targetX, double targetY, double maxPower, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            updatePosition();

            double errorX = targetX - robotX;
            double errorY = targetY - robotY;

            double distanceError = Math.hypot(errorX, errorY);
            if (distanceError < 1) break; // Stop if within 1 inch

            // Calculate movement vector towards target
            double moveAngle = Math.atan2(errorY, errorX);

            // Calculate power components in robot frame
            double robotMoveX = maxPower * Math.cos(moveAngle - robotHeading);
            double robotMoveY = maxPower * Math.sin(moveAngle - robotHeading);

            // Calculate mecanum wheel powers
            double fl = robotMoveY + robotMoveX;
            double fr = robotMoveY - robotMoveX;
            double bl = robotMoveY - robotMoveX;
            double br = robotMoveY + robotMoveX;

            // Normalize powers to max of 1
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            frontLeftMotor.setPower(fl);
            frontRightMotor.setPower(fr);
            backLeftMotor.setPower(bl);
            backRightMotor.setPower(br);

            telemetry.addData("Target (in)", "%.2f, %.2f", targetX, targetY);
            telemetry.addData("Current Pos (in)", "%.2f, %.2f", robotX, robotY);
            telemetry.addData("Distance Error (in)", "%.2f", distanceError);
            telemetry.update();
        }

        stopMotors();
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
