//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Helper.Chassis;

@TeleOp(name = "TeleOpAlaqmar", group = "TeleOp")

public class FieldCentric extends LinearOpMode {
    Chassis chassis;
    double axial;
    double lateral;
    double yaw;


    @Override
    public void runOpMode() {
        chassis = new Chassis();
        chassis.init(this);
        telemetry.addLine("In the runOpMode");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {


            if (gamepad1.a) {
                chassis.resetIMU();
                continue;
            }
            if (gamepad1.b) {
                if (chassis.getDriveMode() == Chassis.DriveMode.ROBOT_CENTRIC) {
                    chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
                } else {
                    chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
                }
                continue;
            }
            axial = -gamepad1.left_stick_y; // Forward/Backward (inverted joystick for push forward = positive)
            lateral = gamepad1.left_stick_x; // Strafe left/right (positive is right, negation is left)
            yaw = gamepad1.right_stick_x; // Turn left/rigth (positive is clockwise, negative is counter-clockwise)


            chassis.drive(axial, lateral, yaw);
            chassis.updateTelemetry();
        }
    }
}
