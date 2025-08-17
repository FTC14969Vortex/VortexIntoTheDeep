package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {

    public CRServo servo;

    public LinearOpMode myOpMode;

    //Constructor
    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() throws InterruptedException {
        servo = myOpMode.hardwareMap.get(CRServo.class, "Intake");
        servo.setDirection(CRServo.Direction.REVERSE);
    }
}
