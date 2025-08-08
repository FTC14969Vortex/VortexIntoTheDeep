package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        double axial;
        double lateral;
        double yaw;

        robot.init(); // This initializes all hardware (chassis, arm, slider, etc.)

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                robot.chassis.odo.resetPosAndIMU(); // Resets the robot’s position and heading if A is pressed
            }

            // ----------------------------- ARM & SLIDER CODE -----------------------------

            // Set motors to run without encoders
            robot.arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slider.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Read input from gamepad2
            double arm_power = gamepad2.left_stick_y;
            double slider_power = gamepad2.right_stick_y;

            // Set motor power
            robot.arm.motor.setPower(arm_power);
            robot.slider.motor.setPower(slider_power);

            // Telemetry output
            telemetry.addData("arm power", arm_power);
            telemetry.addData("slider power", slider_power);

            // ----------------------------- WRIST & INTAKE -----------------------------

// WRIST CONTROL
            double wristScale = 0.01; // Small scale factor for fine control
            double currentWristPos = robot.wrist.servo.getPosition();
            double wristInput = gamepad2.right_stick_x;
            double targetWristPos = currentWristPos + (wristInput * wristScale);

// Clamp the target position between 0 and 1
            targetWristPos = Math.max(0.0, Math.min(1.0, targetWristPos));

// Reset wrist to position 0 if button A is pressed
            if (gamepad2.a) {
                targetWristPos = 0.0;
            }

            robot.wrist.goToPosition(targetWristPos);

// INTAKE CONTROL
            if (gamepad2.left_bumper) {
                robot.intake.setPower(-1.0); // Reverse intake
            } else if (gamepad2.right_bumper) {
                robot.intake.setPower(1.0); // Forward intake
            } else if (gamepad2.x) {
                robot.intake.setPower(0.0); // Stop intake
            }

// Telemetry output
            telemetry.addData("Intake Power", robot.intake.getPower());
// Telemetry output
            telemetry.addData("Wrist Target Pos", targetWristPos);
            telemetry.addData("Intake Power", robot.intake.motor.getPower());

            robot.chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);

            axial = -gamepad1.left_stick_y; // Forward/Backward
            lateral = gamepad1.left_stick_x; // Left/Right
            yaw = gamepad1.right_stick_x; // Rotation

            robot.chassis.drive(axial, lateral, yaw);

            telemetry.update(); // Required to display telemetry values
        }
    }
}