package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Slider {
    public DcMotor motor;
    public int SLIDER_HIGH_BAR_POSITION;
    public int SLIDER_HIGH_BOX_POSITION;
    public int SLIDER_PICK_UP_POSITION;
    double speed = 1;
    int targetPosition;
    int currentPosition;


    LinearOpMode myOpMode;

    //Constructor
    public Slider(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() throws InterruptedException {
        //Init motors and servos
        motor = myOpMode.hardwareMap.get(DcMotor.class, "slider");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void gotoPosition (int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while(motor.isBusy()) {

        }
    }
    public void runWithoutEncoder(int targetTime) {
        ElapsedTime runtime = new ElapsedTime();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speed = speed * Math.signum(targetPosition - currentPosition);
        runtime.reset();
        while (runtime.milliseconds() < targetTime) {
        }
        motor.setPower(0);
    }
    public void gotoSliderHighBoxPosition(){
        this.gotoPosition(SLIDER_HIGH_BOX_POSITION);
    }

    public void gotoSliderHighBarPosition(){
        this.gotoPosition(SLIDER_HIGH_BAR_POSITION);
    }
    public void gotoSliderPickUpPosition(){
        this.gotoPosition(SLIDER_PICK_UP_POSITION);
    }




}
