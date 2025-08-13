package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderDriveOnly", group = "Examples")
public class Encoder extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Encoder constants
    static final double COUNTS_PER_MOTOR_REV = 1120; // Example for Neverest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;  // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 4.0; // Wheel size
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        // Map motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Reverse left side so forward is correct
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run to position mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward 24 inches
            encoderDrive(0.5, 24, 24, 5.0);

            // Turn right (one side forward, one side backward)
            encoderDrive(0.5, 12, -12, 4.0);

            // Drive backward 12 inches
            encoderDrive(0.5, -12, -12, 4.0);
        }
    }

    /**
     * Drive using encoders.
     */
    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newBackLeftTarget  = backLeftDrive.getCurrentPosition()  + (int)(leftInches * COUNTS_PER_INCH);
        int newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        int newBackRightTarget  = backRightDrive.getCurrentPosition()  + (int)(rightInches * COUNTS_PER_INCH);

        // Set targets
        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        backLeftDrive.setTargetPosition(newBackLeftTarget);
        frontRightDrive.setTargetPosition(newFrontRightTarget);
        backRightDrive.setTargetPosition(newBackRightTarget);

        // Start moving
        frontLeftDrive.setPower(Math.abs(speed));
        backLeftDrive.setPower(Math.abs(speed));
        frontRightDrive.setPower(Math.abs(speed));
        backRightDrive.setPower(Math.abs(speed));

        // Wait until motors reach target or stop is requested
        while (opModeIsActive() &&
                (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                        frontRightDrive.isBusy() && backRightDrive.isBusy())) {
            telemetry.addData("Target", "%7d : %7d", newFrontLeftTarget, newFrontRightTarget);
            telemetry.addData("Current", "%7d : %7d", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
