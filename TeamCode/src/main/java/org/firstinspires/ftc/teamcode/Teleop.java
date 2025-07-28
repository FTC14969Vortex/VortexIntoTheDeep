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

            // STEP 1: Set the arm and slider motors to RUN_WITHOUT_ENCODER mode.
            // This tells the motors to respond directly to power input without trying to go to a position.
            // Use the setMode() method and the DcMotor.RunMode.RUN_WITHOUT_ENCODER constant.

            // STEP 2: Read the vertical joystick values from gamepad2.
            // Use gamepad2.left_stick_y for the ARM and gamepad2.right_stick_y for the SLIDER.
            // Save these values in two variables: arm_power and slider_power.

            // STEP 3: Apply power to the motors.
            // Use setPower() on robot.arm.motor and robot.slider.motor using the values you just stored.

            // STEP 4: Display both arm_power and slider_power in the telemetry.
            // This is required so you can monitor how much power is being sent to each motor.
            // Use telemetry.addData("arm power", arm_power); and telemetry.addData("slider power", slider_power);

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
