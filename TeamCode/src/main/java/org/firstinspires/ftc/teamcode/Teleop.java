//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@TeleOp(name = "Dominic TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial;
        float lateral;
        float yaw;
        double max;
        runtime = new ElapsedTime();
        Chassis chassis = new Chassis();
        chassis.init(this);
        chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
        chassis.resetIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                chassis.resetIMU();
            }
            if (gamepad1.x) {
                chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
            } else if (gamepad1.y) {
                chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
            }

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            chassis.drive(axial, lateral, yaw);
            chassis.updateTelemetry();
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

        }
    }
}
