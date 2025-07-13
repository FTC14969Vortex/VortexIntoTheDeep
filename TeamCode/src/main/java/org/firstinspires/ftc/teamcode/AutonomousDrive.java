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
        // Triangle
        myChassis.goToPosition(10, 0, 90, maxPower, timeoutSeconds);
        myChassis.goToPosition(10, 10, 0, maxPower, timeoutSeconds);
        myChassis.goToPosition(0, 0, -45, maxPower, timeoutSeconds);
    }
}