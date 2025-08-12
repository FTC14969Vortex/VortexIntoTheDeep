package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TwoWheelOdometryAuto", group = "Examples")
public class Odometry extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor leftOdo, rightOdo;

    // Odometry constants (update these for your setup)
    static final double TICKS_PER_REV = 8192; // Your encoder ticks per revolution
    static final double WHEEL_DIAMETER_INCHES = 2.0;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    // Distance between left and right odometry wheels in inches (track width)
    static final double TRACK_WIDTH_INCHES = 14.0;

    @Override
    public void runOpMode() {

        // Map drive motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Map odometry wheels
        leftOdo  = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");

        // Reverse left side drive motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset odometry encoders
        resetOdoEncoders();

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward 24 inches
            driveForwardInches(24, 0.4);

            // Turn right 90 degrees
            turnDegrees(90, 0.3);

            // Drive forward 12 inches
            driveForwardInches(12, 0.4);
        }
    }

    // Drive forward by monitoring average of both odo wheels
    private void driveForwardInches(double inches, double power) {
        resetOdoEncoders();
        double targetTicks = inches * TICKS_PER_INCH;

        while (opModeIsActive() &&
                (Math.abs((leftOdo.getCurrentPosition() + rightOdo.getCurrentPosition()) / 2.0) < targetTicks)) {
            setAllDrivePower(power);
            telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
            telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            telemetry.update();
        }
        stopAllMotors();
    }

    // Turn by monitoring difference in odo wheels
    private void turnDegrees(double degrees, double power) {
        resetOdoEncoders();

        // Calculate target encoder ticks for turn
        // Arc length per side = (degrees/360) * (2 * pi * radius)
        // radius = track width / 2
        double turnCircumference = Math.PI * TRACK_WIDTH_INCHES;
        double distancePerSide = (degrees / 360.0) * turnCircumference;
        double targetTicks = distancePerSide * TICKS_PER_INCH;

        while (opModeIsActive() && Math.abs(leftOdo.getCurrentPosition()) < targetTicks) {
            // Turn right: left wheels forward, right wheels backward
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            frontRightDrive.setPower(-power);
            backRightDrive.setPower(-power);

            telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
            telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            telemetry.update();
        }
        stopAllMotors();
    }

    private void resetOdoEncoders() {
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setAllDrivePower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    private void stopAllMotors() {
        setAllDrivePower(0);
    }
}
