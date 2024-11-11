package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Claw {

    public Servo servo;
    private ElapsedTime runtime = new ElapsedTime();
    private int closePosition = 0;
    private int openPosition = 1;


    //Init hardware map
    HardwareMap hwMap = null;

    //Initializing hardware maps and motors for intake.
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        servo = hwMap.get(Servo.class, "Claw");
    }
    //Open
    public void openClaw() {
        servo.setPosition(openPosition);
    }
    //Close
    public void closeClaw() {
    servo.setPosition(closePosition);
    }

}