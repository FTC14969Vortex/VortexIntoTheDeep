//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.Chassis;

@TeleOp(name = "DecodeTeleopV1", group = "TeleOp")

public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis();
        chassis.init(this);
        chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                chassis.resetIMU();

            }
            // If gamepad x is pressed then switch to field centric
            // If gamepad y is pressed then switch to robot centric
            if (gamepad1.x) {
                chassis.resetIMU();
                chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
            } else if (gamepad1.y) {
                chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
            }
            float axial = -gamepad1.left_stick_y;
            float lateral = gamepad1.left_stick_x;
            float yaw = gamepad1.right_stick_x;
            chassis.drive(axial , lateral, yaw);
            chassis.updateTelemetry();
        }
    }

}