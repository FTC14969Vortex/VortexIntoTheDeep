package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "Eric_Auto_ZigZag", group = "Auto")

public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis();
        chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
        chassis.init(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        chassis.updateTelemetry();

        waitForStart();

        // Move chassis autonomously
        double maxPower = 0.2;
        double timeoutSeconds = 10;
        final double step = 60.96; //Each step is 2ft (60.96 cm)

        // Go in a zigzag shape across 4 by 4 mats.
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, 0, step, AngleUnit.DEGREES, 180),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, -step, step, AngleUnit.DEGREES, -90),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, -step, step*2, AngleUnit.DEGREES, 180),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, -step*2, step*2, AngleUnit.DEGREES, -90),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, -step*2, step*3, AngleUnit.DEGREES, 180),
                maxPower, timeoutSeconds);
        chassis.goToPosition(
                new Pose2D(DistanceUnit.CM, -step*3, step*3, AngleUnit.DEGREES, -90),
                maxPower, timeoutSeconds);

    }
}