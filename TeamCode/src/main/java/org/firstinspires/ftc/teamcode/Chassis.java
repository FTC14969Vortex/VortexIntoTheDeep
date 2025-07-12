//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Chassis {
    public Teleop myTeleop;
    public DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    double botHeading;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    public enum opsMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    opsMode currentMode = opsMode.ROBOT_CENTRIC;

    public Chassis(Teleop teleOp) {

        myTeleop = teleOp;

        odo = myTeleop.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        frontLeftDrive = myTeleop.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = myTeleop.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = myTeleop.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = myTeleop.hardwareMap.get(DcMotor.class, "backRightDrive");
    }

    public void setRobotCentric() {
        currentMode = opsMode.ROBOT_CENTRIC;
    }

    public void setFieldCentric() {
        currentMode = opsMode.FIELD_CENTRIC;
    }

    public void init() {

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }
    public void drive(double axial, double lateral, double yaw) {

        if (currentMode == opsMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
            double rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
            double rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
            rotX = rotX * 1.1; //counteract imperfect strafing
            axial = rotY;
            lateral = rotX;
        }

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = (axial - lateral) - yaw;
        double leftBackPower = (axial - lateral) + yaw;
        double rightBackPower = (axial + lateral) - yaw;

        double speed = 1.7;
        double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower))) * speed;

        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }

        // Send calculated power to wheels.
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
    }
}

