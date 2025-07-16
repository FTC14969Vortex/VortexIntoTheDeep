package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "SamSampleZigZagz", group = "Auto")
public class ZigZagAuto extends LinearOpMode {
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
        double timeoutSeconds = 10;
        // Go in a triangle.  Do NOT change this code when you are submitting your homework
        chassis.goToPosition(
                new Pose2D(DistanceUnit.INCH, 0, 22, AngleUnit.DEGREES, 90),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.INCH, 22, 22, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);

        chassis.goToPosition(
                new Pose2D(DistanceUnit.INCH, 22, 44, AngleUnit.DEGREES, -45),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.INCH, 44, 44, AngleUnit.DEGREES, -45),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.INCH, 44, 88, AngleUnit.DEGREES, -45),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.INCH, 88, 88, AngleUnit.DEGREES, -45),
                maxPower, timeoutSeconds);
            }
}