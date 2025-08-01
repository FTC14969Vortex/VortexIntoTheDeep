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
            //To access the arm motor, use robot.arm.motor
            robot.arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slider.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // STEP 2: Use the left joystick on gamepad2 to control the ARM.
            // Get the vertical value of the left stick (Y-axis). Up should be positive, down negative.
            // Save this value in a variable called `arm_power`.
            double arm_power = gamepad2.left_stick_y;
            robot.arm.motor.setPower(arm_power);

            // STEP 3: Use the right joystick on gamepad2 to control the SLIDER.
            // Get the vertical value of the right stick (Y-axis). Up should be positive, down negative.
            // Save this value in a variable called `slider_power`.
            double slider_power = gamepad2.right_stick_y;
            robot.slider.motor.setPower(slider_power);


            // STEP 5: Display the values of `arm_power` and `slider_power`
            // on the telemetry so you can see them on the driver station.
            // Use telemetry.addData("label", value);
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
