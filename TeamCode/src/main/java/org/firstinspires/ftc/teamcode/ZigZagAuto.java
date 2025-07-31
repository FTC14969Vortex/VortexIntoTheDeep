package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "SamZigZagzAuto", group = "Auto")
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
        double[][] vertices = {
                {0, 24, 0},
                {-24, 24, 0},
                {-24, 48, 0},
                {-48, 48, 0},
                {-48, 72, 0},
                {-72, 72, 0}};
        for (double[] vertex : vertices) {
            chassis.goToPosition(
                    new Pose2D(DistanceUnit.INCH, vertex[0], vertex[1], AngleUnit.DEGREES, vertex[2]),
                    maxPower, timeoutSeconds);
        }
    }
}