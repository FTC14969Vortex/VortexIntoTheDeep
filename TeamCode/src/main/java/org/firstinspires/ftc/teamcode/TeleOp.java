//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RobotCentric", group = "TeleOp")

public class TeleOp extends LinearOpMode {

    Chassis chassis = new Chassis();

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial;
        float lateral;
        float yaw;
        double max;

        chassis.init(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            if (gamepad1.x) {
                // Set drive mode to FIELD_CENTRIC
                chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
            } else if (gamepad1.y) {
                // Set drive mode to ROBOT_CENTRIC
                chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
            }
                // If game pad1 a is pressed reset heading
            if (gamepad1.a) {
                chassis.resetIMU();
            }
            chassis.drive(axial, lateral, yaw);
            chassis.updateTelemetry();
        }
    }
}
