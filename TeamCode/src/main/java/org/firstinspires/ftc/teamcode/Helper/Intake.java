package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {

    public CRServo servo1;
    public CRServo servo2;

    public LinearOpMode myOpMode;

    //Constructor
    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Initializing hardware maps and motors for intake.
    public void init() {
        servo1 = myOpMode.hardwareMap.get(CRServo.class, "intake1");
        servo1.setDirection(DcMotorSimple.Direction.REVERSE);
        servo2 = myOpMode.hardwareMap.get(CRServo.class, "intake2");
        servo2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void MoveIntake(double power) {

        servo1.setPower(power);
        servo2.setPower(power);
    }
    public void stopIntake() {
        servo1.setPower(0);
        servo2.setPower(0);
    }

}