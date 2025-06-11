//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {
    Chassis chassis;
    double axial;
    double lateral;
    double yaw;

    @Override
    public void runOpMode() {
        chassis = new Chassis();
        chassis.init(this);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                chassis.resetIMU();
            }
            if (gamepad1.b) {
                if (chassis.getDriveMode() == Chassis.DriveMode.ROBOT_CENTRIC) {
                    chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
                } else {
                    chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
                }
            }
            axial = -gamepad1.left_stick_y; // Forward/Backward (inverted joystick for push forward = positive)
            lateral = gamepad1.left_stick_x; // Strafe left/right (positive is right, negation is left)
            yaw = gamepad1.right_stick_x; // Turn left/rigth (positive is clockwise, negative is counter-clockwise)

            chassis.drive(axial, lateral, yaw);
        }
    }
}