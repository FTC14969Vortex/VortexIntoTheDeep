package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class SampleMechanism {
    public DcMotor motor;

    public SampleMechanism(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "arm");
        motor.setDirection(DcMotor.Direction.FORWARD);
    }

}