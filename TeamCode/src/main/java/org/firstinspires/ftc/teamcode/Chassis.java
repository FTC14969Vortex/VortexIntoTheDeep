package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;


public class Chassis {
    enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    private DcMotor fl, fr, bl, br;
    private OpMode opMode;
    private DriveMode driveMode = DriveMode.FIELD_CENTRIC;
    GoBildaPinpointDriver odo;

    public void init(OpMode opMode) {
        this.opMode = opMode;
        HardwareMap hwMap = opMode.hardwareMap;

        fl = hwMap.get(DcMotor.class, "frontLeftDrive");
        fr = hwMap.get(DcMotor.class, "frontRightDrive");
        bl = hwMap.get(DcMotor.class, "backLeftDrive");
        br = hwMap.get(DcMotor.class, "backRightDrive");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        odo = hwMap.get(GoBildaPinpointDriver.class,"odo");

        odo.recalibrateIMU();
        odo.resetPosAndIMU();

    }

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }

    public void updateTelemetry() {
        opMode.telemetry.addData("Drive Mode", driveMode);
        opMode.telemetry.update();
    }

    public void drive(double axial, double lateral, double yaw) {
        double heading = 0;

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            heading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
            opMode.telemetry.addLine("botHeading " + heading);
        }

        double rotX = lateral * Math.cos(heading) - axial * Math.sin(heading);
        double rotY = lateral * Math.sin(heading) + axial * Math.cos(heading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);

        double flPower = (rotY + rotX + yaw)/denominator;
        double frPower = (rotY - rotX - yaw)/denominator;
        double blPower = (rotY - rotX + yaw)/denominator;
        double brPower = (rotY + rotX - yaw)/denominator;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
}


