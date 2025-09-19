package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class FlyWheel {

    
    private DcMotor flyWheel;
    private OpMode opMode;

    public void init (OpMode opMode) {

        this.opMode = opMode;

        flyWheel = opMode.hardwareMap.get(DcMotor.class, "flyWheel");


    }

    public void start(double power) {

        flyWheel.setPower(power);
    }
    public void stop() {

        flyWheel.setPower(0);
}
    }