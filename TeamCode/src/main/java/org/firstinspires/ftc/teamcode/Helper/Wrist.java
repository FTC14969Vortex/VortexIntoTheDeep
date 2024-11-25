package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo servo;
    //servo in use is 5 turn: 0.01 change ~ 5 degrees.
    public double TARGET_POSITION;

    public double WRIST_DELIVERY_POSITION_HIGH = 0.2;
    public double WRIST_DELIVERY_POSITION_LOW = 0.22;
    public double WRIST_DELIVERY_POSITION_AUTO = 0.2;
    public double WRIST_PICKUP_POSITION = 0.309;

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

    public void servoPosition(double TARGET_POSITION) {
        servo.setPosition(TARGET_POSITION);

    }

    public void gotoPickupPosition(){
        servo.setPosition(WRIST_PICKUP_POSITION);
    }
    public void gotoLowPosition(){
        servo.setPosition(WRIST_DELIVERY_POSITION_LOW);
    }
    public void gotoHighPosition(){
        servo.setPosition(WRIST_DELIVERY_POSITION_HIGH);
    }
    public void gotoAutoPosition(){
        servo.setPosition(WRIST_DELIVERY_POSITION_AUTO);
    }

    public void gotoPosition(double currPos) {servo.setPosition(currPos); }



}
