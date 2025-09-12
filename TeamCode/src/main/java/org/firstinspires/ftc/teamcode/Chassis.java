package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Chassis {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    public Chassis(HardwareMap hardwareMap) {

        leftFront=hardwareMap.get(DcMotor.class, "leftFront");
        leftBack=hardwareMap.get(DcMotor.class, "leftBack");
        rightFront=hardwareMap.get(DcMotor.class, "rightFront");
        rightBack=hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }
}
