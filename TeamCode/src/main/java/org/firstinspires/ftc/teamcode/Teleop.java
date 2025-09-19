//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.Chassis;
import org.firstinspires.ftc.teamcode.helper.FlyWheel;
import org.firstinspires.ftc.teamcode.helper.Gate;

@TeleOp(name = "DecodeTeleopV2.4", group = "TeleOp")

public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis();
        chassis.init(this);
        chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

        FlyWheel flyWheel = new FlyWheel();
        flyWheel.init(this);

        Gate gate = new Gate();
        gate.init(this);

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

            if (gamepad1.right_bumper) {
                flyWheel.start(1);
                sleep(1500);
                gate.release(1);

            } else if (gamepad1.left_bumper) {
                flyWheel.stop();
                gate.stop();


        }
    }

}
    }