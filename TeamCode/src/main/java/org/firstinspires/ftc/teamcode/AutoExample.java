package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helper.Robot;


@Autonomous(name = "Auto Example", group = "Auto")
public class AutoExample extends LinearOpMode{
    //------------------------------------------------------------------

    //Robot Object
    public Robot robot = new Robot();

    // Robot control parameters
    double DRIVE_SPEED = 0.8;
    float TURN_OFFSET = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code here
        robot.init(hardwareMap);


        waitForStart();
        // Code to run after the driver hits PLAY

//        // Drive forward
//        robot.chassis.Drive(DRIVE_SPEED, 30);
//        sleep(1000);
//
//        // Drive backward
//        robot.chassis.Drive(DRIVE_SPEED, -30);
//        sleep(1000);
//
//        // Strafe left
//        robot.chassis.Strafe(DRIVE_SPEED, -30);
//        sleep(1000);
//
//        // Strafe right
//        robot.chassis.Strafe(DRIVE_SPEED, 30);
//        sleep(1000);
//
//        // Turn right
//        robot.chassis.autoTurn(90, TURN_OFFSET);
//        sleep(1000);
//
//        // Turn left
//        robot.chassis.autoTurn(-90, TURN_OFFSET);
//        sleep(1000);
//
//        // Stop
//        robot.chassis.stopDriveMotors();


        //Move arm and wrist to delivery position
        robot.arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.gotoAutoPosition();
        sleep(100);
        robot.wrist.gotoAutoPosition();
        sleep(1500);

        // Open the gate to deliver one pixel.
        robot.gate.open();
        sleep(1000);

        // Bring the wrist and arm to pickup position.
        robot.wrist.gotoPosition(robot.wrist.WRIST_PICKUP_POSITION);
        sleep(850);
        robot.arm.gotoPickupPosition();

        // Close the gate.
        robot.gate.close();
        sleep(1000);



//        //Run the arm without encoder for 1 second.
//        robot.arm.runWithoutEncoder(5000);
//        sleep(1000);

    }
}
