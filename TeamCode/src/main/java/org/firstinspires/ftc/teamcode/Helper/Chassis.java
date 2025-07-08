package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

public class Chassis {
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC; //default to Robot Centric

    private DcMotor FLMotor;
    private DcMotor BLMotor;
    private DcMotor FRMotor;
    private DcMotor BRMotor;
    // The IMU sensor object
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;

    private OpMode botOpMode;
    private HardwareMap hardwareMap;
    //Telemetry telemetry;
    private double botHeading;
public void init(OpMode opMode) {
    botOpMode = opMode;
    hardwareMap = opMode.hardwareMap;
    //telemetry = opMode.telemetry;
    botOpMode.telemetry.addData("Status", "Start initialization");

    FLMotor = hardwareMap.get(DcMotor.class, "frontLeftDrive");
    BLMotor = hardwareMap.get(DcMotor.class, "backLeftDrive");
    FRMotor = hardwareMap.get(DcMotor.class, "frontRightDrive");
    BRMotor = hardwareMap.get(DcMotor.class, "backRightDrive");
    odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

    FLMotor.setDirection(DcMotor.Direction.REVERSE);
    BLMotor.setDirection(DcMotor.Direction.REVERSE);
    FRMotor.setDirection(DcMotor.Direction.FORWARD);
    BRMotor.setDirection(DcMotor.Direction.FORWARD);

    // Recalibrate IMU
    odo.recalibrateIMU();
    odo.resetPosAndIMU();
    botOpMode.telemetry.addData("Status", "Initialized");
    botOpMode.telemetry.update();
}
public DriveMode getDriveMode() {
        return this.driveMode;
}

public void setDriveMode(DriveMode mode) {
    driveMode = mode;
    botOpMode.telemetry.addData("Set drive mode to:", mode.toString() );
    botOpMode.telemetry.update();
} // gamepad B to switch between Robot Centric and Field Centric modes

public void resetIMU() {
    odo.resetPosAndIMU();
    botOpMode.telemetry.addData("Reset IMU", "Reset robot heading 0" );
    botOpMode.telemetry.update();
} // gamepad A to reset robot heading to 0

// Print out
// - Driving Mode
// - Odometry computer's reading of (heading, x, y)
public void updateTelemetry() {
    odo.update();
    botOpMode.telemetry.addData("Drive Mode", driveMode.toString());
    botOpMode.telemetry.addData("Bot Heading", JavaUtil.formatNumber(odo.getHeading(AngleUnit.DEGREES), 4, 2));
    botOpMode.telemetry.addData("BotX", JavaUtil.formatNumber(odo.getPosX(DistanceUnit.CM), 4, 2));
    botOpMode.telemetry.addData("BotY", JavaUtil.formatNumber(odo.getPosY(DistanceUnit.CM), 4, 2));
    botOpMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
    botOpMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
    botOpMode.telemetry.update();
}


// TeleOp Mode Methods
/**
 * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
 * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
 * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
 */
public void drive(double axial, double lateral, double yaw) {

    if (driveMode == DriveMode.ROBOT_CENTRIC) {
    /*
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = (axial - lateral) - yaw;
        leftBackPower = (axial - lateral) + yaw;
        rightBackPower = (axial + lateral) - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }
        // Send calculated power to wheels.
        FLMotor.setPower(leftFrontPower);
        FRMotor.setPower(rightFrontPower);
        BLMotor.setPower(leftBackPower);
        BRMotor.setPower(rightBackPower);
        return;
    */

        // Unified way for Robot Centric
        botHeading = 0;

    } else if (driveMode == DriveMode.FIELD_CENTRIC) {
        odo.update();
        botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
     }

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

    // Show the telemetry data.
    updateTelemetry();
}

}
