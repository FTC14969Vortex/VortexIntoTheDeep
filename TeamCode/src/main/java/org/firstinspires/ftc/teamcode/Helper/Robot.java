package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Related to IMU


public class Robot {

    public Arm arm;
    public Wrist wrist;
    public Slider slider;
    public Chassis chassis;
    public Intake intake;

    //Constructor for robot class
    public Robot(LinearOpMode opMode) {
        arm = new Arm(opMode);
        wrist = new Wrist(opMode);
        slider = new Slider(opMode);
        chassis = new Chassis(opMode);
        intake = new Intake(opMode);
    }

    public void init() throws InterruptedException {
        arm.init();
        wrist.init();
        slider.init();
        chassis.init();
        intake.init();
    }
}
