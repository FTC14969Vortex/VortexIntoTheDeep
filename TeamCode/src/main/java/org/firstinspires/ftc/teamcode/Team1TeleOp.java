package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Basic TeleOp", group = "Linear Opmode")
public class Team1TeleOp extends LinearOpMode {
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(300);
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Tank drive control
            double forward_and_backwardsPower  = 0.3 * -gamepad1.left_stick_y;
            double side_to_sidePower = 0.3 * -gamepad1.left_stick_x;
            //frontLeftDrive.setPower(leftPower);
            frontLeftDrive.setPower(forward_and_backwardsPower);
            backLeftDrive.setPower(forward_and_backwardsPower);
            frontRightDrive.setPower(forward_and_backwardsPower);
            backRightDrive.setPower(forward_and_backwardsPower);
            frontLeftDrive.setPower(side_to_sidePower);
            backLeftDrive.setPower(side_to_sidePower);
            frontRightDrive.setPower(side_to_sidePower);
            backRightDrive.setPower(side_to_sidePower);
            telemetry.addData("Forward and Backwards Power", forward_and_backwardsPower);
            telemetry.addData("Side to Side Power", side_to_sidePower);
            telemetry.update();
        }
    }
}
