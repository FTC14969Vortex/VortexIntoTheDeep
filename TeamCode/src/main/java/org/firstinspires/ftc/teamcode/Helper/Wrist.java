package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo servo;
    //servo in use is 5 turn: 0.01 change ~ 5 degrees.
    public double TARGET_POSITION;
//
    public double WRIST_HANG_POS = 1;
    public double WRIST_MID_POS = 0.66;


    public LinearOpMode myOpMode;

    //Constructor
    public Wrist(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() throws InterruptedException {
        //Init motors and servos
        servo = myOpMode.hardwareMap.get(Servo.class, "wrist");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void turnToHangPos() {
        gotoPosition(WRIST_HANG_POS);
    }

    public void turnToMidPos() {
        gotoPosition(WRIST_MID_POS);
    }

    public void gotoPosition(double currPos) {servo.setPosition(currPos); }



}
