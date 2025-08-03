package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class  Arm {

    //Object creation
    public DcMotor motor;
    double speed = 1;
    int targetPosition;
    int currentPosition;

    LinearOpMode myOpMode;

    //Constructor
    public Arm(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() throws InterruptedException {
        //Init motors and servos
        motor = myOpMode.hardwareMap.get(DcMotor.class, "Arm");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void moveArm(double power) {
        motor.setPower(power);
    }

    public void gotoPosition (int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while(motor.isBusy()) {}
    }
}