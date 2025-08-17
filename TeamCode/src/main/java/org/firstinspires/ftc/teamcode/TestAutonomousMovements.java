package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Helper.Intake;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TestAutonomousMovements9;")
public class TestAutonomousMovements extends LinearOpMode {

    Chassis chassis = null;
    Intake intake = null;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;


    // Reference to GoBilda's Pinpoint odometry driver
    public GoBildaPinpointDriver odo;


    // Drive motor references
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new Chassis();
        chassis.init(this);

        intake = new Intake(this);
        intake.init();



        waitForStart();

        if (opModeIsActive()) {

            chassis.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            resetRuntime();

           // sleep(3000);
            //telemetry.addData("Running turnToHeading(): ", 180);
            //telemetry.update();
            //chassis.turnToHeading(90,0.8,5000);

            //sleep(3000);
            //telemetry.addData("Running turnWithPIDControl(): ", 180);
            //telemetry.update();
            //chassis.turnWithPIDControl(90);

            //sleep(3000);
            //telemetry.addData("Running turnWithProportionalControl(): ", 180);
            //telemetry.update();
            //chassis.turnWithProportionalControl(90);

            //telemetry.addData("Start turn180(): ", 180);
            //telemetry.update();
            //chassis.turn180();
            //telemetry.addData("End turn180(): ", 180);

            //sleep(3000);

            backLeftDrive.setPower(0.8);
            frontLeftDrive.setPower(0.8);
            backRightDrive.setPower(-0.8);
            frontRightDrive.setPower(-0.8);

            sleep(1500);

            backLeftDrive.setPower(0);
            frontLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);

            for (int i = 0; i < 1 ; i++) {
//                chassis.moveByTime(Chassis.Direction.FORWARD, .5, 2);
//                chassis.moveByTime(Chassis.Direction.BACKWARD, .5, 2);
//
//                sleep(5000);
//
//                chassis.moveWithEncoder(Chassis.Direction.FORWARD, .5, 60);
//                chassis.moveWithEncoder(Chassis.Direction.BACKWARD, .5, 58);
//
//                sleep(5000);
//
                //intake.servo.setPower(1);
                //chassis.moveWithProportionalDeceleration(Chassis.Direction.FORWARD, .8, 30);
                //sleep(1000);
               //intake.servo.setPower(0);
                //chassis.moveWithProportionalDeceleration(Chassis.Direction.BACKWARD, .8, 30);
                //sleep(500);
                //chassis.setRobotPowerToWheels(0.8,0.8,-0.8,-0.8);
                //sleep(500);
               // chassis.moveWithProportionalDeceleration(Chassis.Direction.FORWARD, 0.8, 30);
               // intake.servo.setPower(-1);


//                sleep(5000);

                //chassis.moveWithProportionalDecelerationAndHeading( Chassis.Direction.FORWARD, .8, 60, 0.1);
                //chassis.moveWithProportionalDecelerationAndHeading( Chassis.Direction.BACKWARD, .8, 58, 0.1);

            }

            //telemetry.addData("Run Time: ", getRuntime());
            //telemetry.update();
            //sleep(10000);



            double inch2mm = 25.4;
            double driveSpeed = 0.5;
            int timeoutMs = 3000;

            Pose2D startPose = chassis.getPoseEstimate();
            double x0 = startPose.getX(DistanceUnit.MM);
            double y0 = startPose.getY(DistanceUnit.MM);
            double heading = startPose.getHeading(AngleUnit.RADIANS);

            // Define a path of 4 positions
//            List<Pose2D> path = Arrays.asList(
//                    new Pose2D(DistanceUnit.MM, x0, y0 + 3 * inch, AngleUnit.RADIANS, heading),        // Forward
//                    new Pose2D(DistanceUnit.MM, x0 + 3 * inch, y0 + 3 * inch, AngleUnit.RADIANS, heading), // Right
//                    new Pose2D(DistanceUnit.MM, x0, y0 + 3 * inch, AngleUnit.RADIANS, heading),        // Left
//                    new Pose2D(DistanceUnit.MM, x0, y0, AngleUnit.RADIANS, heading)                    // Back
//            );
//
//            // Follow the path
//            chassis.followPath(path, power, timeoutMs);
//            chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0, y0 + 36 * inch2mm, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
//            chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0 + 36 * inch2mm, y0, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
////            chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0, y0 - 36 * inch2mm, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
//            chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0 - 36 * inch2mm, y0, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
//            //chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0 -24 * inch2mm, y0 + 24, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
            //chassis.stop();
        }
    }
}