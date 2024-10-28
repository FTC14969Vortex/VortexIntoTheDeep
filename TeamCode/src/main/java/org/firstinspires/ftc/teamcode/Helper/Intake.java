package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {

    public CRServo servo;
    private ElapsedTime runtime = new ElapsedTime();

    //Init hardware map
    HardwareMap hwMap = null;

    //Initializing hardware maps and motors for intake.
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        servo = hwMap.get(CRServo.class, "intake");
    }

    public void MoveIntake(double position) {
        servo.setPower(position);
    }
}