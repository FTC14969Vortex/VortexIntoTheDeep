package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Sophia_Autonomous_Mecanum", group = "Auto")
public class Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis();
        chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
        chassis.init(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Move chassis autonomously
        double maxPower = 0.5;
        double timeoutSeconds = 30;
        // Go in a triangle.  Do NOT change this code when you are submitting your homework
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, 0, 60.96, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, 60.96, 60.96, AngleUnit.DEGREES, -90),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, 60.96, 121.92, AngleUnit.DEGREES, 90),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, 121.92, 121.92, AngleUnit.DEGREES, -90),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, 121.92, 182.88, AngleUnit.DEGREES, 90),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, 182.88, 182.88, AngleUnit.DEGREES, -90),
                maxPower, timeoutSeconds);
    }
}