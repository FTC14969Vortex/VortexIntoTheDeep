package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo servo;
    //servo in use is 5 turn: 0.01 change ~ 5 degrees.
//
    public double WRIST_HANG_POS = 0.5861;
    public double WRIST_PICK_UP_POS = 0.66;
    public double WRIST_STRAIGHT_POS = 0.4567;
    public double WRIST_VERTICAL_DOWN = 0.1139;
    public double WRIST_TOP_FOLD = 1;


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

    public void turnToPickUpPos() {
        gotoPosition(WRIST_PICK_UP_POS);
    }
    public void turnToStraightPosition(){gotoPosition(WRIST_STRAIGHT_POS);}
    public void goToHighBox(){gotoPosition(WRIST_VERTICAL_DOWN);}

    public void gotoPosition(double currPos) {servo.setPosition(currPos);  }



}
