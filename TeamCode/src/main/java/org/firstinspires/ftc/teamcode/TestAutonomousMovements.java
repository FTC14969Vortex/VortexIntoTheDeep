package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.Chassis;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TestAutonomousMovements2;")
public class TestAutonomousMovements extends LinearOpMode {

    Chassis chassis = null;

    @Override
    public void runOpMode() {

        chassis = new Chassis();
        chassis.init(this);

        waitForStart();

        if (opModeIsActive()) {

            chassis.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            chassis.setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            resetRuntime();
            for (int i = 0; i < 1 ; i++) {
                //chassis.moveByTime(Chassis.Direction.FORWARD, .5, 2);
                //chassis.moveByTime(Chassis.Direction.BACKWARD, .5, 2);

                //chassis.moveWithEncoder(Chassis.Direction.FORWARD, .5, 64);
                //chassis.moveWithEncoder(Chassis.Direction.BACKWARD, .5, 64);

                chassis.moveWithProportionalDeceleration(Chassis.Direction.FORWARD, .8, 60);
                chassis.moveWithProportionalDeceleration(Chassis.Direction.BACKWARD, .8, 58);
            }

            telemetry.addData("Run Time: ", getRuntime());
            telemetry.update();
            sleep(10000);

            //Move by time
            //chassis.moveByTime(Chassis.Direction.FORWARD, .5, 2);

            //Move using encoder
            //chassis.moveWithEncoder(Chassis.Direction.FORWARD, .5, 64);



            //double inch2mm = 25.4;
            //double driveSpeed = 0.2;
            //int timeoutMs = 3000;

            //Pose2D startPose = chassis.getPoseEstimate();
            //double x0 = startPose.getX(DistanceUnit.MM);
            //double y0 = startPose.getY(DistanceUnit.MM);
            //double heading = startPose.getHeading(AngleUnit.RADIANS);

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
            //chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0, y0 + 12 * inch2mm, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
           // chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0 + 36 * inch2mm, y0, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
            //chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0, y0 - 36 * inch2mm, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
            //chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0 - 36 * inch2mm, y0, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
            //chassis.goToPosition(new Pose2D(DistanceUnit.MM, x0 -24 * inch2mm, y0 + 24, AngleUnit.RADIANS, heading), driveSpeed, timeoutMs);
            //chassis.stop();
        }
    }
}