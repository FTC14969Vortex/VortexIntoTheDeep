package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "OdoIMUBasedAuto", group = "Examples")
public class odometry_imu extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor odo; // Odometry wheel motor/encoder
    private BNO055IMU imu;

    // Constants for odometry
    static final double TICKS_PER_REV = 8192; // Rev Through Bore encoder example
    static final double WHEEL_DIAMETER_INCHES = 2.0;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    @Override
    public void runOpMode() {

        // Map motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Map odometry wheel
        odo = hardwareMap.get(DcMotor.class, "odo");

        // Map IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // Reverse left side motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset odometry
        odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("IMU Calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addLine("IMU Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example: Move forward 24 inches
            moveForwardInches(24, 0.4);

            // Example: Turn to 90 degrees
            turnToHeading(90, 0.3);

            // Example: Move forward another 12 inches
            moveForwardInches(12, 0.4);

            stopAllMotors();
        }
    }

    // ===== Movement Methods =====
    private void moveForwardInches(double inches, double power) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);
        odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (inches > 0) {
            while (opModeIsActive() && odo.getCurrentPosition() < targetTicks) {
                setAllDrivePower(power);
                telemetry.addData("Odo Position", odo.getCurrentPosition());
                telemetry.update();
            }
        } else {
            while (opModeIsActive() && odo.getCurrentPosition() > targetTicks) {
                setAllDrivePower(-power);
                telemetry.addData("Odo Position", odo.getCurrentPosition());
                telemetry.update();
            }
        }
        stopAllMotors();
    }

    private void turnToHeading(double targetHeading, double power) {
        double currentHeading = getHeading();
        double error = angleError(targetHeading, currentHeading);

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            double turnPower = Math.copySign(power, error);
            setTurnPower(turnPower);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target", targetHeading);
            telemetry.update();

            currentHeading = getHeading();
            error = angleError(targetHeading, currentHeading);
        }
        stopAllMotors();
    }

    // ===== Helper Methods =====
    private void setAllDrivePower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    private void setTurnPower(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(-power);
    }

    private void stopAllMotors() {
        setAllDrivePower(0);
    }

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    // Ensures shortest turn direction
    private double angleError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }
}
