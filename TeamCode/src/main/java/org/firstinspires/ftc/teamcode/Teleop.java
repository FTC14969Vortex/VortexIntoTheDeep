//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@TeleOp(name = "TeleOp: Molly", group = "TeleOp")

public class Teleop extends LinearOpMode {





    @Override
    public void runOpMode() {
        float axial;
        float lateral;
        float yaw;
        Chassis chassis = new Chassis();
        chassis.init(this);

        chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);

        waitForStart();

    //    runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                chassis.resetIMU();
            }
            if (gamepad1.b) {
                chassis.toggleDriveMode();
            }
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            chassis.drive(axial, lateral, yaw);








        }
    }
}
