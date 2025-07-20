package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "SampleAuto_Mecanum_Alaqmar", group = "Auto")
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
        double feetToCM = 30.48;

         {
             //Go straight
            Pose2D pose2D0 = new Pose2D(DistanceUnit.CM, 0,  1 * feetToCM, AngleUnit.DEGREES, 0);
            chassis.goToPosition( pose2D0 , maxPower , timeoutSeconds );

            // Go left
             Pose2D pose2D1 = new Pose2D(DistanceUnit.CM, 1 * -feetToCM,1 * feetToCM, AngleUnit.DEGREES, -90);
             chassis.goToPosition( pose2D1, maxPower , timeoutSeconds );

             //Go straight
             Pose2D pose2D2 = new Pose2D(DistanceUnit.CM, 1 * -feetToCM,  2 * feetToCM, AngleUnit.DEGREES, 0);
             chassis.goToPosition( pose2D2 , maxPower , timeoutSeconds );

             // Go left
             Pose2D pose2D3 = new Pose2D(DistanceUnit.CM, 2 * -feetToCM,  2 * feetToCM, AngleUnit.DEGREES, -90);
             chassis.goToPosition( pose2D3 , maxPower , timeoutSeconds );

             //Go straight
             Pose2D pose2D4 = new Pose2D(DistanceUnit.CM, 2 * -feetToCM,  3 * feetToCM, AngleUnit.DEGREES, 0);
             chassis.goToPosition( pose2D4 , maxPower , timeoutSeconds );

             // Go left
             Pose2D pose2D5 = new Pose2D(DistanceUnit.CM, 3 * -feetToCM,  3 * feetToCM, AngleUnit.DEGREES, -90);
             chassis.goToPosition( pose2D5 , maxPower , timeoutSeconds );
         }
    }
}