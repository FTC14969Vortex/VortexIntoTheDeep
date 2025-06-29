package org.firstinspires.ftc.teamcode.Helper;

//imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Chassis {
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    double frontLeftPower;
    double backLeftPower;
    double frontrightPower;
    double backRightPower;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DriveMode driveMode;
    private OpMode opMode;

    public void init(OpMode opMode) {
        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        // Recalibrate IMU
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void setDriveMode(DriveMode mode) {
        this.driveMode = driveMode;
    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }

    public void updateTelemetry() {
//        opMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
//        opMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
//        opMode.telemetry.update();
    }

    public void drive(double axial, double lateral, double yaw) {
       double botHeading;
       if (driveMode == DriveMode.FIELD_CENTRIC) {
           odo.update();
           botHeading = -odo.getHeading(AngleUnit.RADIANS);
       }
       else {
           botHeading = 0;
       }


       double max;
       double lateral_1 = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
       double axial_1 = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);


       frontLeftPower = axial_1 + lateral_1 + yaw;
       frontrightPower = axial_1 - lateral_1 - yaw;
       backLeftPower = axial_1 - lateral_1 + yaw;
       backRightPower = axial_1 + lateral_1 - yaw;

       max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontrightPower), Math.abs(backLeftPower),Math.abs(backRightPower)));
       if  (max > 1) {
           frontLeftPower = frontLeftPower / max;
           frontrightPower = frontrightPower / max;
           backLeftPower = backLeftPower / max;
           backRightPower = backRightPower / max;
       }
       frontLeftDrive.setPower(frontLeftPower);
       frontRightDrive.setPower(frontrightPower);
       backLeftDrive.setPower(backLeftPower);
       backRightDrive.setPower(backRightPower);


    }
}