//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.Arm;
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

            // ----------------------------- STUDENT SECTION START -----------------------------

            // Set arm motor to RUN_USING_ENCODER for active holding
            robot.arm.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.slider.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Slider stays manual

            // Read joystick input
            double rawArmInput = -gamepad2.left_stick_y;
            double rawSliderInput = -gamepad2.right_stick_y;

            // Deadzone to prevent twitching
            double armDeadzone = 0.05;
            double sliderDeadzone = 0.1;
            double arm_power = Math.abs(rawArmInput) > armDeadzone ? rawArmInput * 0.3 : 0.001;
            double slider_power = Math.abs(rawSliderInput) > sliderDeadzone ? rawSliderInput : 0;

            // Apply power
            robot.arm.motor.setPower(arm_power);
            robot.slider.motor.setPower(slider_power);

            // Telemetry for debugging
            telemetry.addData("Arm Power", arm_power);
            telemetry.addData("Slider Power", slider_power);
// ------------------------------ STUDENT SECTION END ------------------------------

            robot.chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);

            axial = -gamepad1.left_stick_y; // Forward/Backward
            lateral = gamepad1.left_stick_x; // Left/Right
            yaw = gamepad1.right_stick_x; // Rotation

            robot.chassis.drive(axial, lateral, yaw);

            telemetry.update(); // Don't forget: telemetry won't show anything unless you call update()!
        }
    }
}
