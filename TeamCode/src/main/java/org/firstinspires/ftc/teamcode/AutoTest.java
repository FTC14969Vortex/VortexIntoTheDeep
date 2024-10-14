package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helper.Robot;


@Autonomous(name = "Auto Test", group = "Auto")
@Disabled
public class AutoTest extends LinearOpMode{
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

        // Drive forward
        robot.chassis.Drive(DRIVE_SPEED, 30);
        sleep(1000);

        // Drive backward
        robot.chassis.Drive(DRIVE_SPEED, -30);
        sleep(1000);

        // Strafe left
        robot.chassis.Strafe(DRIVE_SPEED, -30);
        sleep(1000);

        // Strafe right
        robot.chassis.Strafe(DRIVE_SPEED, 30);
        sleep(1000);

        // Turn right
        robot.chassis.autoTurn(90, TURN_OFFSET);
        sleep(1000);

        // Turn left
        robot.chassis.autoTurn(-90, TURN_OFFSET);
        sleep(1000);

        // Stop
        robot.chassis.stopDriveMotors();
    }
}
