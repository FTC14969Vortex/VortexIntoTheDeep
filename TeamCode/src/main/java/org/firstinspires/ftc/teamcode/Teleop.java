package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "TeleOp_Competition", group = "TeleOp")
public class Teleop extends LinearOpMode {

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        double axial;
        double lateral;
        double yaw;

        final double chassis_high_speed = 0.9;
        final double chassis_low_speed = 0.5;

        final int arm_high_position = 3885;
        final int arm_low_position = 100;

        final int slider_high_position = -1800;
        final int slider_low_position = 50;

        final double wrist_center = 0.48;
        final double wrist_range = 0.08;

        robot.init(); // This initializes all hardware (chassis, arm, slider, etc.)

        telemetry.setAutoClear(false);
        Telemetry.Item TeleChassisSpeed = telemetry.addData("ITEM: Chassis Speed", 0);
        Telemetry.Item TeleArmPosition = telemetry.addData("ITEM: Arm Position", 0);
        Telemetry.Item TeleSliderPosition = telemetry.addData("ITEM: Slider Position", 0);

        robot.wrist.servo.scaleRange(wrist_center - wrist_range, wrist_center + wrist_range);

        robot.chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

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
            double arm_power = -gamepad2.left_stick_y;
            double slider_power = gamepad2.right_stick_y;

            // Set motor power
            robot.arm.motor.setPower(arm_power * 0.8);
            robot.slider.motor.setPower(slider_power * 0.8);

            if(gamepad2.dpad_up){
                robot.arm.runToPosition(arm_high_position);
                robot.slider.runToPosition(slider_high_position);
            }

            if(gamepad2.dpad_down){
                robot.slider.runToPosition(slider_low_position);
                robot.arm.runToPosition(arm_low_position);
            }

            //telemetry.addData("arm power", arm_power);
            //telemetry.addData("slider power", slider_power);
            //telemetry.addData("arm position", robot.arm.motor.getCurrentPosition());
            //telemetry.addData("slider position", robot.slider.motor.getCurrentPosition());
            TeleArmPosition.setValue(robot.arm.motor.getCurrentPosition());
            TeleSliderPosition.setValue(robot.slider.motor.getCurrentPosition());
            telemetry.update();


            // ----------------------------- WRIST & INTAKE -----------------------------

            // WRIST CONTROL:
            // Use right_stick_x on gamepad2 to rotate the wrist slightly left/right.
            // 1. Get the current wrist position using robot.wrist.servo.getPosition()
            // 2. Add the gamepad joystick value to that current position value. You may need to multiply by a small scale factor.
            // 3. Call goToPosition on the wrist to move it to the gamepad + current value.
            //
            // BONUS: If gamepad2.a is pressed, reset the wrist to position 0.

            double current_position = robot.wrist.servo.getPosition();
            double wrist_input = -gamepad2.right_stick_x;
            robot.wrist.gotoPosition(current_position + wrist_input * 0.02);

            if (gamepad2.a) {
                robot.wrist.gotoPosition(wrist_center);
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

            //telemetry.addData("Wrist Position", robot.wrist.servo.getPosition());
            //telemetry.addData("Intake Power", robot.intake.servo.getPower());
            //telemetry.update();

            // ----------------------------------------------------------------------------------


            // switch drive mode
            if (gamepad1.b) {
                Chassis.DriveMode currentDriveMode = robot.chassis.getDriveMode();
                robot.chassis.setDriveMode(currentDriveMode == Chassis.DriveMode.ROBOT_CENTRIC ?
                        Chassis.DriveMode.FIELD_CENTRIC : Chassis.DriveMode.ROBOT_CENTRIC);
            }

            if (gamepad1.x) { // set to low speed mode
                robot.chassis.speedFactor = chassis_low_speed;
            }
            if (gamepad1.y) { // set to high speed mode
                robot.chassis.speedFactor = chassis_high_speed;
            }

            if (gamepad1.dpad_up || gamepad1.right_bumper) {
                robot.chassis.speedFactor = Math.min(robot.chassis.speedFactor + robot.chassis.speedStep, 1.0);
            }
            if (gamepad1.dpad_down || gamepad1.left_bumper) {
                robot.chassis.speedFactor = Math.max(robot.chassis.speedFactor - robot.chassis.speedStep, 0.1);
            }


            axial = -gamepad1.left_stick_y; // Forward/Backward
            lateral = gamepad1.left_stick_x; // Left/Right
            yaw = gamepad1.right_stick_x; // Rotation

            robot.chassis.drive(axial, lateral, yaw);

            //robot.chassis.updateTelemetry();

            TeleChassisSpeed.setValue(robot.chassis.speedFactor);
            telemetry.update(); // Required to display telemetry values
        }
    }
}
