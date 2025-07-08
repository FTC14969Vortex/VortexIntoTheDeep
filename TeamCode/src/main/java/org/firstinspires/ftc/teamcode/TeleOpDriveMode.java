//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@TeleOp(name = "Eric TeleOp DriveMode", group = "TeleOp")

public class TeleOpDriveMode extends LinearOpMode {

    Chassis team1Chassis;
    double axial;
    double lateral;
    double yaw;

    @Override
    public void runOpMode() {
        team1Chassis = new Chassis();
        team1Chassis.init(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                team1Chassis.resetIMU();
            }

            if (gamepad1.b) {
                Chassis.DriveMode currentDriveMode = team1Chassis.getDriveMode();
                team1Chassis.setDriveMode(currentDriveMode == Chassis.DriveMode.ROBOT_CENTRIC ?
                        Chassis.DriveMode.FIELD_CENTRIC : Chassis.DriveMode.ROBOT_CENTRIC);
            }

            /*
            if (gamepad1.x) {
                team1Chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
            }
            if (gamepad1.y) {
                team1Chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
            }
            */

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // stick Y direction: up is -, down is +
            // stick X direction: left is -, right is +
            axial = -gamepad1.left_stick_y; // Forward/Backward (inverted joystick for push forward = positive)
            lateral = gamepad1.left_stick_x; // Strafe left/right (positive is right, negation is left)
            yaw = gamepad1.right_stick_x; // Turn left/rigth (positive is clockwise, negative is counter-clockwise)

            team1Chassis.drive(axial, lateral, yaw);
        }
    }
}