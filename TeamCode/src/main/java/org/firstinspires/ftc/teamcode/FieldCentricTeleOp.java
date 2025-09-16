package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helper.Chassis;

public class FieldCentricTeleOp extends LinearOpMode {

    Chassis chassis;

    public void runOpMode() {
        chassis = new Chassis();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            chassis.drive(y, x, rx);





        }



    }
}