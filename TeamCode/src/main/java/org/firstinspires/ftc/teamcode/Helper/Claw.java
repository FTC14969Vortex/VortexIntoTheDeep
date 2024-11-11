package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Claw {
    public Servo servo;

    public double Open = 0;
    public double Close = 1;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        servo = hwMap.get(Servo.class, "Claw");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void open() {
        servo.setPosition(Open);
    }

    public void close() {
        servo.setPosition(Close);
    }


}