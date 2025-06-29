package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Chassis {
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DriveMode driveMode = DriveMode.FIELD_CENTRIC;
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

    public void init(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
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
    }
    public void setDriveMode(DriveMode mode) {
        this.driveMode = mode;
        telemetry.addData("Changing driveMode to:", mode.toString() );
        telemetry.update();
    }

    public DriveMode getDriveMode() {
        return this.driveMode;
    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }
    // Print out
    // - Driving Mode
    // - Odometry computer's reading of (heading, x, y)
    public void updateTelemetry() {
        // Wait for the game to start (driver presses START)
        telemetry.addData("DriveMode : ", driveMode.toString());
        telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
        telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
        telemetry.update();
    }
    // TeleOp Mode Methods
    /**
     * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
     * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
     * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
     */
    public void drive(double axial, double lateral, double yaw) {
        odo.update();
        double botHeading =0;
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
            telemetry.addLine("botHeading " + botHeading);
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

        FLMotor.setPower(leftBackPower);
        FRMotor.setPower(rightFrontPower);
        BLMotor.setPower(leftBackPower);
        BRMotor.setPower(rightBackPower);
    }
}
