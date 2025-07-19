package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "SampleAuto_Mecanum_Armaan", group = "Auto")
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
        double timeoutSeconds = 30;
        double twoFeetToCM = 2*30.48;

        for (int i = 0; i < 3; i++) {

            
            // Go to (0, 24) with 0 degrees rotation
            chassis.goToPosition(
                    new Pose2D(DistanceUnit.CM, i * -twoFeetToCM, (i+1) * twoFeetToCM, AngleUnit.DEGREES, 0),
                    maxPower, timeoutSeconds);

            // Go to (24, 24) with -90 degrees rotation
            chassis.goToPosition(
                    new Pose2D(DistanceUnit.CM, (i+1)* -twoFeetToCM, (i+1) *twoFeetToCM, AngleUnit.DEGREES, -90),
                    maxPower, timeoutSeconds);

        }
    }
}