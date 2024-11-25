package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {

    public CRServo servo;
    public LinearOpMode myOpMode;

    //Constructor
    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Initializing hardware maps and motors for intake.
    public void init() {
        servo = myOpMode.hardwareMap.get(CRServo.class, "intake");
    }

    public void MoveIntake(double position) {
        servo.setPower(position);
    }

}