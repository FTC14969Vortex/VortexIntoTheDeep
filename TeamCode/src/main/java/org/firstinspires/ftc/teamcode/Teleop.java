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

            // WRIST CONTROL:
            // Use right_stick_x on gamepad2 to rotate the wrist slightly left/right.
            // 1. Get the current wrist position using robot.wrist.servo.getPosition()
            // 2. Add the gamepad joystick value to that current position value. You may need to multiply by a small scale factor.
            // 3. Call goToPosition on the wrist to move it to the gamepad + current value.
            //
            // BONUS: If gamepad2.a is pressed, reset the wrist to position 0.

            // INTAKE CONTROL:
            // - gamepad2 left_bumper → reverse intake (power = -1)
            // - gamepad2 right_bumper → forward intake (power = 1)
            // - gamepad2 x → stop intake (power = 0)

            // ----------------------------------------------------------------------------------

            robot.chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);

            axial = -gamepad1.left_stick_y; // Forward/Backward
            lateral = gamepad1.left_stick_x; // Left/Right
            yaw = gamepad1.right_stick_x; // Rotation

            robot.chassis.drive(axial, lateral, yaw);

            telemetry.update(); // Required to display telemetry values
        }
    }
}