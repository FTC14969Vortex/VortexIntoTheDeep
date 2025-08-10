package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto_Encoder_IMU_Mecanum", group = "Auto")
public class AutoEncoderIMU extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // Odometry wheels (2 encoders)
    private DcMotor leftOdo, rightOdo;

    // IMU sensor
    private BNO055IMU imu;

    // Constants
    private static final double ODO_TICKS_PER_REV = 8192; // Example: REV Through-Bore Encoder
    private static final double ODO_WHEEL_DIAMETER_INCHES = 2.0;
    private static final double TICKS_PER_INCH = ODO_TICKS_PER_REV / (Math.PI * ODO_WHEEL_DIAMETER_INCHES);

    // PID constants for heading correction
    private static final double HEADING_KP = 0.05;

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

        // IMU initialization parameters
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParams);

        // Reverse motors if needed
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset odometry encoders
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward 24 inches while keeping heading 0 degrees
            driveForwardWithIMU(24, 0.5, 0);

            // Turn 90 degrees clockwise
            turnToAngle(90);

            // Strafe right 24 inches (approximate)
            strafeRight(24, 0.5);

            stopMotors();
        }
    }

    /**
     * Drives forward a certain distance (inches) using odometry encoder average and IMU heading correction.
     */
    private void driveForwardWithIMU(double inches, double power, double targetAngle) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);
        int startLeft = leftOdo.getCurrentPosition();
        int startRight = rightOdo.getCurrentPosition();

        while (opModeIsActive()) {
            int leftTicks = Math.abs(leftOdo.getCurrentPosition() - startLeft);
            int rightTicks = Math.abs(rightOdo.getCurrentPosition() - startRight);
            int avgTicks = (leftTicks + rightTicks) / 2;

            if (avgTicks >= targetTicks) break;

            double currentHeading = imu.getAngularOrientation().firstAngle;
            double error = targetAngle - currentHeading;
            double correction = error * HEADING_KP;

            double leftPower = power - correction;
            double rightPower = power + correction;

            // For mecanum forward, vy=0, vx=power, omega=correction (adjusted here)
            // But since mecanum wheels, apply same power to all 4 motors with correction on left/right sides
            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(rightPower);

            telemetry.addData("Driving Forward", "%.2f inches", inches);
            telemetry.addData("Current Ticks", avgTicks);
            telemetry.addData("Heading", currentHeading);
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * Turns robot to a target heading using IMU and proportional control.
     */
    private void turnToAngle(double targetAngle) {
        double error;
        double power;
        do {
            double currentHeading = imu.getAngularOrientation().firstAngle;
            error = targetAngle - currentHeading;

            power = error * 0.02; // P control constant for turning
            power = Math.max(Math.min(power, 0.5), -0.5); // limit power

            // Set mecanum rotation: vx=0, vy=0, omega=power
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(-power);

            telemetry.addData("Turning to angle", targetAngle);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.update();

        } while (opModeIsActive() && Math.abs(error) > 2); // 2 degree tolerance

        stopMotors();
    }

    /**
     * Strafes right approx distance (inches) using drive motor encoders and IMU heading correction.
     * NOTE: strafing with only two odometry wheels is approximate.
     */
    private void strafeRight(double inches, double power) {
        // Using frontLeft and frontRight encoders for strafe distance (approximate)
        int targetTicks = (int) (inches * TICKS_PER_INCH);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            int leftTicks = Math.abs(frontLeftMotor.getCurrentPosition());
            int rightTicks = Math.abs(frontRightMotor.getCurrentPosition());
            int avgTicks = (leftTicks + rightTicks) / 2;

            if (avgTicks >= targetTicks) break;

            double currentHeading = imu.getAngularOrientation().firstAngle;
            double error = 0 - currentHeading; // keep heading 0 during strafe
            double correction = error * HEADING_KP;

            // Mecanum strafe right: vx=0, vy=power, omega=correction
            double flPower = power - correction;
            double frPower = -power + correction;
            double blPower = -power - correction;
            double brPower = power + correction;

            frontLeftMotor.setPower(flPower);
            frontRightMotor.setPower(frPower);
            backLeftMotor.setPower(blPower);
            backRightMotor.setPower(brPower);

            telemetry.addData("Strafing Right", "%.2f inches", inches);
            telemetry.addData("Current Ticks", avgTicks);
            telemetry.addData("Heading", currentHeading);
            telemetry.update();
        }
        stopMotors();
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
