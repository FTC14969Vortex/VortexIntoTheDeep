//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "TeleopAlicia", group = "TeleOp")

public class Teleop extends LinearOpMode {

    public Chassis myChassis = new Chassis(this);

    @Override
    public void runOpMode() {

        myChassis.init();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.x) {
                myChassis.setRobotCentric();
            }
            if (gamepad1.y) {
                myChassis.setFieldCentric();
            }
            if (gamepad1.a) {
                myChassis.resetIMU();
            }

            myChassis.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

            telemetry.update();

        }
    }
}