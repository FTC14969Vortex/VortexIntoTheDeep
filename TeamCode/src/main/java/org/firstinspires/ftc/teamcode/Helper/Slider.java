package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slider {
    public CRServo servo;
    public double SLIDER_HIGH_BAR_POSITION;
    public double SLIDER_HIGH_BOX_POSITION;


    public LinearOpMode myOpMode;

    //Constructor
    public Slider (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() throws InterruptedException {
        //Init motors and servos
        servo = myOpMode.hardwareMap.get(CRServo.class, "slider");
        servo.setDirection(CRServo.Direction.FORWARD);
    }
   
    public void MoveSlider(double power) {

        servo.setPower(power);
    }


}
