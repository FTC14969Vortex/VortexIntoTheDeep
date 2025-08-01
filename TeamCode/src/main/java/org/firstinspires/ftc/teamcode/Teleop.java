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

            // STEP 1: Set motors to RUN_WITHOUT_ENCODER mode
            // This allows direct power control without targeting a specific position
                    robot.arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.slider.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // STEP 2: Read joystick input from gamepad2
            // Negative sign ensures pushing up gives positive power
                    double rawArmInput = -gamepad2.left_stick_y;     // Left stick controls arm
                    double rawSliderInput = -gamepad2.right_stick_y; // Right stick controls slider

            // STEP 3: Apply deadzone to prevent small joystick movements from activating motors
            // This avoids twitching when the stick is near center
                    double deadzone = 0.05;
                    double arm_power = Math.abs(rawArmInput) > deadzone ? rawArmInput : 0;
                    double slider_power = Math.abs(rawSliderInput) > deadzone ? rawSliderInput : 0;

            // STEP 4: Scale power to limit speed and protect hardware
            // You can adjust these values based on how strong or fragile your mechanism is
                    double armScale = 0.4;    // Limits arm to 40% power
                    double sliderScale = 0.4; // Limits slider to 40% power
                    arm_power *= armScale;
                    slider_power *= sliderScale;

            // STEP 5: Send scaled power to motors
                    robot.arm.motor.setPower(arm_power);
                    robot.slider.motor.setPower(slider_power);

            // STEP 6: Display power values on telemetry for debugging
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
