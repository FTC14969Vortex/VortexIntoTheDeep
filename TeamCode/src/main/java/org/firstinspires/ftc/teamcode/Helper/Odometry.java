package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Odometry Auto (No IMU)", group = "Odometry")
public class Odometry extends LinearOpMode {

    private Chassis chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init chassis
        chassis = new Chassis(this);
        chassis.init();

        // Disable any IMU reset
        // We will NOT call chassis.resetIMU();

        telemetry.addLine("Odometry-only Initialized - Waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Start position will be whatever odometry reads at start
            // Movement sequence example (coordinates in CM, heading in DEGREES from odometry only)
            goTo(new Pose2D(DistanceUnit.CM, 50, 0, AngleUnit.DEGREES, 0), 0.3, 5);   // Forward 50 cm
            goTo(new Pose2D(DistanceUnit.CM, 50, 50, AngleUnit.DEGREES, 90), 0.3, 5); // Right 50 cm, face 90°
            goTo(new Pose2D(DistanceUnit.CM, 0, 50, AngleUnit.DEGREES, 180), 0.3, 5); // Back to X=0, rotate to 180°
            goTo(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0), 0.3, 5);    // Return to origin

            telemetry.addLine("Path complete");
            telemetry.update();
            sleep(2000);
        }
    }

    /**
     * Helper method for movement
     */
    private void goTo(Pose2D target, double maxPower, double timeout) {
        chassis.goToPosition(target, maxPower, timeout);
    }
}
