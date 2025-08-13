package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TimeBasedMecanumAuto", group = "Examples")
public class timebased extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    @Override
    public void runOpMode() {

        // Map motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Reverse directions so forward is forward
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {

            // 1. Drive forward for 2 seconds
            driveForward(0.5);
            sleep(1000);

            // Stop for 0.5 seconds
            stopAllMotors();
            sleep(1000);

            // 2. Strafe right for 1.5 seconds
            strafeRight(0.5);
            sleep(1000);

            // Stop for 0.5 seconds
            stopAllMotors();
            sleep(500);

            // 3. Turn right for 1 second
            turnRight(0.5);
            sleep(1000);

            // Stop all motors
            stopAllMotors();
        }
    }

    // ===== Helper methods =====

    private void driveForward(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    private void driveBackward(double power) {
        driveForward(-power);
    }

    private void strafeRight(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }

    private void strafeLeft(double power) {
        strafeRight(-power);
    }

    private void turnRight(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(-power);
    }

    private void turnLeft(double power) {
        turnRight(-power);
    }

    private void stopAllMotors() {
        driveForward(0);
    }
}
