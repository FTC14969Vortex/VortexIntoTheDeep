package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp7", group = "TeleOp")
public class Teleop extends LinearOpMode {

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        double axial;
        double lateral;
        double yaw;

        robot.init(); // This initializes all hardware (chassis, arm, slider, etc.)

        robot.wrist.servo.scaleRange(0.40, 0.55);

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
            double arm_power = - gamepad2.left_stick_y;
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

            double current_position = robot.wrist.servo.getPosition();
            telemetry.addData("current_wrist_position", current_position);
            double wrist_input = gamepad2.right_stick_x;
            robot.wrist.gotoPosition(current_position + wrist_input * 0.02);

            if (gamepad2.a) {
                robot.wrist.gotoPosition(0.48);
            }

            // INTAKE CONTROL:
            // - gamepad2 left_bumper → reverse intake (power = -1)
            // - gamepad2 right_bumper → forward intake (power = 1)
            // - gamepad2 x → stop intake (power = 0)

            if (gamepad2.left_bumper) {
                robot.intake.servo.setPower(-1);
            }
            if (gamepad2.right_bumper) {
                robot.intake.servo.setPower(1);
            }
            if (gamepad2.x) {
                robot.intake.servo.setPower(0);
            }

            // ----------------------------------------------------------------------------------

            robot.chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

            axial = 0.6 * -gamepad1.left_stick_y; // Forward/Backward
            lateral = 0.6 * gamepad1.left_stick_x; // Left/Right
            yaw = 0.7 * gamepad1.right_stick_x; // Rotation

            robot.chassis.drive(axial, lateral, yaw);

            telemetry.update(); // Required to display telemetry values
        }
    }
}
