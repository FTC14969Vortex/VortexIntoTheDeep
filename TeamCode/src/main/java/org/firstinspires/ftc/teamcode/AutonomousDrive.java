//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Autonomous(name = "AutonomousAlicia", group = "Autonomous")

public class AutonomousDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        Chassis myChassis = new Chassis(this);
        myChassis.setFieldCentric();
        myChassis.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Move chassis autonomously
        double maxPower = 0.5;
        double timeoutSeconds = 10;
        // Move up to the left corner
        myChassis.goToPosition( 0, 60.96, 0, maxPower, timeoutSeconds);
        myChassis.goToPosition(-60.96, 60.96, -90, maxPower, timeoutSeconds);
        myChassis.goToPosition(-60.96, 121.96, 0, maxPower, timeoutSeconds);
        myChassis.goToPosition( -121.92, 121.92, -90, maxPower, timeoutSeconds);
        myChassis.goToPosition(-121.92, 182.88, 0, maxPower, timeoutSeconds);
        myChassis.goToPosition(-182.88, 182.88, -90, maxPower, timeoutSeconds);
    }
}