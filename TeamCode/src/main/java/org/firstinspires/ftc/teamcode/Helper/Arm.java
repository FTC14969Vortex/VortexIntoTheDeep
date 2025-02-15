package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class  Arm {

    //Object creation
    public DcMotor motor;
    int timeout_ms = 5000;
    double speed = 1;
    int targetPosition;
    int currentPosition;
    public int ARM_BOX_DELIVERY_POSITION_LOW = 3000;
    public int ARM_BAR_DELIVERY_POSITION_HIGH1 = 3150;
    public int ARM_BAR_DELIVERY_POSITION_HIGH2 = 3150;
    public int ARM_PICKUP_POSITION = 4950;

    public int ARM_MID_POSITION = 2400;
    public int ARM_BAR_DELIVERY_POSITION_LOW = 4100;
//    int slowDown;

    public double ARM_HOLDING_POWER = 0.05;

    LinearOpMode myOpMode;

    //Constructor
    public Arm(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() throws InterruptedException {
        //Init motors and servos
        motor = myOpMode.hardwareMap.get(DcMotor.class, "swingArm");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    // ARM WITH BUTTONS V2

    public void gotoPosition (int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while(motor.isBusy()) {

        }
    }
//    public void gotoPosition(int targetPosition) {
//        ElapsedTime runtime = new ElapsedTime();
//        timeout_ms = 3000;
//        currentPosition = motor.getCurrentPosition();
//        motor.setTargetPosition(targetPosition);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        speed = speed * Math.signum(targetPosition - currentPosition);
//        //Set the power of the motor.
//        motor.setPower(speed);
//        runtime.reset();
//
//        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
//        }
//
//        if(motor.getCurrentPosition() > ARM_MID_POSITION) {
//            motor.setPower(-ARM_HOLDING_POWER); //Holding power.
//        } else {
//            motor.setPower(ARM_HOLDING_POWER);
//        }
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//    }

    public void runWithoutEncoder(int targetTime) {
        ElapsedTime runtime = new ElapsedTime();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speed = speed * Math.signum(targetPosition - currentPosition);
        runtime.reset();
        while (runtime.milliseconds() < targetTime) {
        }
        motor.setPower(0);
    }

    public void gotoPickUpPosition(){
        this.gotoPosition(ARM_PICKUP_POSITION);
    }

    public void gotoHighBar1(){
        this.gotoPosition(ARM_BAR_DELIVERY_POSITION_HIGH1);
    }
    public void gotoHighBar2(){
        this.gotoPosition(ARM_BAR_DELIVERY_POSITION_HIGH2);
    }
    public void gotoLowBar(){
        this.gotoPosition(ARM_BAR_DELIVERY_POSITION_LOW);
    }

    public void gotoLowBox() { this.gotoPosition(ARM_BOX_DELIVERY_POSITION_LOW); }

    public void gotoMidPosition() { this.gotoPosition(ARM_MID_POSITION); }


}