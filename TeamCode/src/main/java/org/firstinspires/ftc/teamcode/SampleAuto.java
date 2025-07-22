package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "SampleAuto_Mecanum_Amelia", group = "Auto")
public class SampleAuto extends LinearOpMode {
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

        double x = 0;
        double y = 0;
        /* for (int i = 0; i >= 2; i++); {
            y += 25;
            chassis.goToPosition(
                    new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                    maxPower, timeoutSeconds);
            x -= 25;
            chassis.goToPosition(
                    new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                    maxPower, timeoutSeconds);
        }*/
        y += 60.96;
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);
        x -= 60.96;
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);
        y += 60.96;
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);
        x -= 60.96;
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);
        y += 60.96;
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);
        x -= 60.96;
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, 0),
                maxPower, timeoutSeconds);
    }
}