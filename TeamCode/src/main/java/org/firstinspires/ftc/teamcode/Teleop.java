//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {

    Robot robot = new Robot(this);

    //How fast your robot will accelerate.
    public double ACCELERATION = 0.3;

    //Motor powers
    public double fl_power = 0;
    public double bl_power = 0;
    public double fr_power = 0;
    public double br_power = 0;

    public double DRIVETRAIN_SPEED = 0.5;
    @Override
    public void runOpMode() throws InterruptedException{
        /**
         * Instance of Robot class is initialized
         */
        robot.init();

        /**
         * This code is run during the init phase, and when opMode is not active
         * i.e. When "INIT" Button is pressed on the Driver Station App
         */


        robot.chassis.FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.chassis.BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.chassis.FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.chassis.BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            /**
             * Joystick controls for Drivetrain, Intake on GAMEPAD 1
             */

            //Drive train


            //open and close claw function
            if(gamepad1.a){
                robot.claw.open();
            }
            if(gamepad1.b){
                robot.claw.close();
            }
            // Controller to motor powers.
            double move_y_axis = gamepad1.left_stick_y;
            double move_x_axis = -gamepad1.left_stick_x;
            double pivot_turn = -gamepad1.right_stick_x;


            //Sets the target power
            double target_fl_power = move_y_axis + move_x_axis + pivot_turn;
            double target_bl_power = move_y_axis - move_x_axis + pivot_turn;
            double target_fr_power = move_y_axis - move_x_axis - pivot_turn;
            double target_br_power = move_y_axis + move_x_axis - pivot_turn;

            //Adds how far you are from target power, times acceleration to the current power.
            fl_power += ACCELERATION * (target_fl_power - fl_power);
            bl_power += ACCELERATION * (target_bl_power - bl_power);
            fr_power += ACCELERATION * (target_fr_power - fr_power);
            br_power += ACCELERATION * (target_br_power - br_power);


            robot.chassis.FLMotor.setPower(DRIVETRAIN_SPEED * fl_power);
            robot.chassis.BLMotor.setPower(DRIVETRAIN_SPEED * bl_power);
            robot.chassis.FRMotor.setPower(DRIVETRAIN_SPEED * fr_power);
            robot.chassis.BRMotor.setPower(DRIVETRAIN_SPEED * br_power);

            //Outake
            if(gamepad2.left_trigger != 0) {
                robot.intake.MoveIntake(-gamepad2.left_trigger);
            }
            //Intake
            if(gamepad2.right_trigger != 0) {
                robot.intake.MoveIntake(gamepad2.right_trigger);
            }
            //stop the intake
            if(gamepad2.x) {
                robot.intake.servo.setPower(0);
            }


            robot.wrist.gotoPosition(robot.wrist.servo.getPosition() + gamepad2.right_stick_x * 0.01);
            /**
             * Joystick controls for Slider, Arm, Wrist, Gate on GAMEPAD 2
             */

            double swing_arm_power = -gamepad2.left_stick_y * 0.7;


            // Running without encoder allows the arm to be swung from current position.
            robot.arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm.motor.setPower(swing_arm_power);
            // Set arm, wrist, and gate to pickup or delivery position with bumper.

            //Gate
            if(gamepad2.x){
                robot.claw.close();
            }
            if(gamepad2.y){
                robot.claw.open();
            }

            YawPitchRollAngles imu = robot.chassis.imu.getRobotYawPitchRollAngles();
            //Telemetry
            telemetry.addData("FL Motor Encoder", robot.chassis.FLMotor.getCurrentPosition());
            telemetry.addData("BL Motor Encoder", robot.chassis.BLMotor.getCurrentPosition());
            telemetry.addData("BR Motor Encoder", robot.chassis.BRMotor.getCurrentPosition());
            telemetry.addData("FR Motor Encoder", robot.chassis.FRMotor.getCurrentPosition());
            telemetry.addData("Yaw", imu.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", imu.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", imu.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Arm Position", robot.arm.motor.getCurrentPosition());
            telemetry.addData("Motor Status", robot.arm.motor.isBusy());
            telemetry.addData("Arm Power", robot.arm.motor.getPower());
            telemetry.update();
        }




        telemetry.addData("FL Motor Encoder", robot.chassis.FLMotor.getCurrentPosition());
        telemetry.addData("BL Motor Encoder", robot.chassis.BLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Encoder", robot.chassis.BRMotor.getCurrentPosition());
        telemetry.addData("FR Motor Encoder", robot.chassis.FRMotor.getCurrentPosition());
        Orientation angle;
        angle = robot.chassis.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("Angular Orientation", angle);
        telemetry.addData("IsRobotStable", robot.chassis.isRobotStable());


        telemetry.update();

    }

}