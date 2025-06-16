//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

@TeleOp(name = "TeleopSophia", group = "TeleOp")

public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        double axial;
        double lateral;
        double yaw;
        Chassis chassis = new Chassis();
        chassis.init(this);

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                chassis.resetIMU();
            }

            if (gamepad1.x) {
                    chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
                    telemetry.addLine("FIELD_CENTRIC");
            }
            if (gamepad1.y) {
                chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
                telemetry.addLine("ROBOT_CENTRIC");
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // stick Y direction: up is -, down is +
            // stick X direction: left is -, right is +
            axial = -gamepad1.left_stick_y; // Forward/Backward (inverted joystick for push forward = positive)
            lateral = gamepad1.left_stick_x; // Strafe left/right (positive is right, negation is left)
            yaw = gamepad1.right_stick_x; // Turn left/rigth (positive is clockwise, negative is counter-clockwise)

            chassis.drive(axial,lateral,yaw);
        }
    }
}