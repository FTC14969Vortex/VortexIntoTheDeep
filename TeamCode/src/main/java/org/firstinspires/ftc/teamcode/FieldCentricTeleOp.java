package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class FieldCentricTeleOp extends LinearOpMode {

    Chassis chassis;

    public void runOpMode() {
        chassis = new Chassis(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            drive(y, x, rx);





        }



    }
}