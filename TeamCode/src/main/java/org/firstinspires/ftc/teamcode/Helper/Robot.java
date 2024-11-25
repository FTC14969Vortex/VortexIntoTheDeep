package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Related to IMU


public class Robot {
    /*
    Properties that describe hardware.
     */
    private ElapsedTime runtime = new ElapsedTime();

    private LinearOpMode myOpMode;


    //Contructor for robot class
    public Robot(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public Chassis chassis = new Chassis(myOpMode);
    public Intake intake = new Intake(myOpMode);
    public Arm arm = new Arm(myOpMode);
    public Wrist wrist = new Wrist(myOpMode);
    public Claw claw = new Claw(myOpMode);

    public void init() throws InterruptedException {
        chassis.init(true);
        intake.init();
        arm.init();
        wrist.init();
        claw.init();
    }
}
