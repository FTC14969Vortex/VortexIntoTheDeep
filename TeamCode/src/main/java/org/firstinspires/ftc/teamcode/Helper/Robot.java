package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Related to IMU


public class Robot {

    public Chassis chassis;
    public Intake intake;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;

    //Constructor for robot class
    public Robot(LinearOpMode opMode) {
        chassis = new Chassis(opMode);
        intake = new Intake(opMode);
        arm = new Arm(opMode);
        wrist = new Wrist(opMode);
        claw = new Claw(opMode);
    }

    public void init() throws InterruptedException {
        chassis.init(true);
        intake.init();
        arm.init();
        wrist.init();
        claw.init();
    }
}
