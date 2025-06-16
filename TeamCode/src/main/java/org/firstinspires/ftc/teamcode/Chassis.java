//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

public class Chassis {
    // The Motor objects
    private DcMotor FLMotor;
    private DcMotor BLMotor;
    private DcMotor FRMotor;
    private DcMotor BRMotor;
    // The IMU sensor object
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    DriveMode driveMode;
    OpMode opMode;

    public void init(OpMode mode){
        opMode = mode;
        driveMode = DriveMode.ROBOT_CENTRIC;
        FLMotor = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        BLMotor = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        FRMotor = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        BRMotor = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);


        // Recalibrate IMU
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        // Wait for the game to start (driver presses START)
        updateTelemetry();
    }
    public void setDriveMode(DriveMode mode){
        driveMode = mode;
    }  // gamepad B

    public void resetIMU() {
        odo.resetPosAndIMU();
        odo.update();
    }                    // gamepad A
    // Print out
    // - Driving Mode
    // - Odometry computer's reading of (heading, x, y)
    public void updateTelemetry() {
        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
        opMode.telemetry.addLine("botHeading " + -odo.getHeading(AngleUnit.RADIANS));
        opMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
        opMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
        opMode.telemetry.addData("botX", JavaUtil.formatNumber(odo.getPosX(DistanceUnit.CM), 4, 2));
        opMode.telemetry.addData("botY", JavaUtil.formatNumber(odo.getPosY(DistanceUnit.CM), 4, 2));
        opMode.telemetry.update();
    }
    // TeleOp Mode Methods
    /**
     * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
     * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
     * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
     */
    public void drive(double axial, double lateral, double yaw){
        odo.update();
        double botHeading = 0;
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
        }

        updateTelemetry();


        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
        rotX = rotX * 1.1; //counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double speed = 1.7;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1) * speed; //Multiply by 1.7 to reduce speed
        leftFrontPower = (rotY + rotX + yaw) / denominator;
        rightFrontPower = (rotY - rotX - yaw) / denominator;
        leftBackPower = (rotY - rotX + yaw) / denominator;
        rightBackPower = (rotY + rotX - yaw) / denominator;

        // Send calculated power to wheels.
        FLMotor.setPower(leftFrontPower);
        FRMotor.setPower(rightFrontPower);
        BLMotor.setPower(leftBackPower);
        BRMotor.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime);
        updateTelemetry();
    }
}