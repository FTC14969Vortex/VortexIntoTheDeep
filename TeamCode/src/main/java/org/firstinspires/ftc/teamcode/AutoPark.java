/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helper.Robot;


/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Auto Park", group = "Auto")

public class AutoPark extends LinearOpMode
{
    // get an instance of the "Robot" class.
    public Robot robot = new Robot(this);

    @Override public void runOpMode() throws InterruptedException {
        // Initialize the robot hardware & Turn on telemetry
        robot.init();

        // Wait for driver to press start
        while(!isStarted()) {
            if(opModeInInit()) {
                robot.chassis.readSensors();
                telemetry.addData(">", "Touch Play to run Auto");
                telemetry.update();
             }
        }

        waitForStart();
        robot.chassis.resetHeading();  // Reset heading to set a baseline for Auto
        robot.chassis.drive(-48, 0.8,0.25);
        // Run Auto if stop was not pressed.
//
    }
}