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

@TeleOp(name = "TeleOp_Molly", group = "TeleOp")
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

            // STEP 1: Set the arm and slider motors to RUN_WITHOUT_ENCODER mode.
            // This tells the motors to respond directly to power input without trying to go to a position.
            // Use the setMode() method and the DcMotor.RunMode.RUN_WITHOUT_ENCODER constant.
            //To access the arm motor, use robot.arm.motor
            robot.arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slider.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // STEP 2: Use the left joystick on gamepad2 to control the ARM.
            // Get the vertical value of the left stick (Y-axis). Up should be positive, down negative.
            // Save this value in a variable called `arm_power`.
            double arm_power = -gamepad2.left_stick_y;
            robot.arm.motor.setPower(arm_power);

            if (arm_power == 0) {
                robot.arm.motor.setPower(0.1);
            }else {
                robot.arm.motor.setPower(arm_power);
            }

            // STEP 3: Use the right joystick on gamepad2 to control the SLIDER.
            // Get the vertical value of the right stick (Y-axis). Up should be positive, down negative.
            // Save this value in a variable called `slider_power`.
            double slider_power = -gamepad2.right_stick_y;
            robot.slider.motor.setPower(slider_power);

            // STEP 5: Display the values of `arm_power` and `slider_power`
            // on the telemetry so you can see them on the driver station.
            // Use telemetry.addData("label", value);
            this.telemetry.addData("arm_power: ", arm_power);
            this.telemetry.addData("slider_power: ", slider_power);
            // ----------------------------- WRIST & INTAKE -----------------------------

            // WRIST CONTROL:
            // Use right_stick_x on gamepad2 to rotate the wrist slightly left/right.
            // 1. Get the current wrist position using robot.wrist.servo.getPosition()
            double current_wrist_position = robot.wrist.servo.getPosition();
            // 2. Add the gamepad joystick value to that current position value. You may need to multiply by a small scale factor.
            double target_wrist_position = (gamepad2.right_stick_x * 0.2) + current_wrist_position;

            // 3. Call goToPosition on the wrist to move it to the gamepad + current value.
            robot.wrist.gotoPosition(target_wrist_position);
            //
            // BONUS: If gamepad2.a is pressed, reset the wrist to position 0.
            if (gamepad2.a){
                robot.wrist.gotoPosition(0);
            }
            // INTAKE CONTROL:
            // - gamepad2 left_bumper → reverse intake (power = -1)
            // - gamepad2 right_bumper → forward intake (power = 1)
            // - gamepad2 x → stop intake (power = 0)
            if (gamepad2.left_bumper) {
                robot.intake.servo.setPower(-1);
            } else if (gamepad2.right_bumper) {
                robot.intake.servo.setPower(1);
            } else if (gamepad2.x) {
                robot.intake.servo.setPower(0);
            }
            telemetry.addData("current_wrist_position", current_wrist_position);
            telemetry.addData("target_wrist_position", target_wrist_position);
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
