package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "AutonomousDrive-Peter", group = "Auto")
public class AutonomousDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(this);
        chassis.setFieldCentric();
        chassis.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Move chassis autonomously
        double maxPower = 0.5;
        double timeoutSeconds = 10;
        // Go in a triangle.  Do NOT change this code when you are submitting your homework
        chassis.goToPosition(10, 0,90, maxPower, timeoutSeconds);
        chassis.goToPosition(10, 10,0, maxPower, timeoutSeconds);
        chassis.goToPosition(0, 0,-45, maxPower, timeoutSeconds);
    }
}