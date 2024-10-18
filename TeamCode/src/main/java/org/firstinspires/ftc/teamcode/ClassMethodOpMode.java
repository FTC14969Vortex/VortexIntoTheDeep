package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class ClassMethodOpMode extends OpMode {

    @Override
    public void init() {
        Point.dooy();
    }


    public Point output;
        public ClassMethodOpMode() {
            char x_in = 'c';
            output = new Point(x_in);
        }


    double squareInputWithSign(double input) {
        double output = input * input;
        if (input < 0) {
            output = output * -1;
        }
        return output;
    }

    @Override
    public void loop() {
        double leftAmount = gamepad1.left_stick_x;
        double fwdAmount = -gamepad1.left_stick_y;
        telemetry.addData("Before X", leftAmount);
        telemetry.addData("Before Y", fwdAmount);
        leftAmount = squareInputWithSign(leftAmount);
        fwdAmount = squareInputWithSign(fwdAmount);
        telemetry.addData("After X", leftAmount);
        telemetry.addData("After Y", fwdAmount);
    }
}