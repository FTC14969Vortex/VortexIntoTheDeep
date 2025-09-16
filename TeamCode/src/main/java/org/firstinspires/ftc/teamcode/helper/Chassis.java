package org.firstinspires.ftc.teamcode.helper;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Chassis {
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private OpMode opMode;
    private DriveMode driveMode;

    public enum DriveMode {

        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public void init(OpMode opMode) {

        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightBack");
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void setDriveMode(DriveMode driveMode) {
        // save driveMode to use in drive()
        this.driveMode = driveMode;

    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }

    public void updateTelemetry() {
        // opMode.telemetry.addData("Status", "Run Time: " + runtime);
        opMode.telemetry.addData("Drive Mode", driveMode);
        opMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
        opMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
        odo.update();
        opMode.telemetry.addData("botHeading", JavaUtil.formatNumber(odo.getHeading(AngleUnit.RADIANS), 4, 2));
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
    public void drive(double axial, double lateral, double yaw) {
        // If in field centric mode read botHeading from odo otherwise set botHeading equal to zero
        double botHeading;
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = odo.getHeading(AngleUnit.RADIANS);// Get the robot's heading in radians

        } else {
            botHeading = 0;
        }


        /*// Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
        rotX = rotX * 1.1; //counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double speed = 1.7;
        leftFrontPower = (rotY + rotX + yaw);
        rightFrontPower = (rotY - rotX - yaw);
        leftBackPower = (rotY - rotX + yaw);
        rightBackPower = (rotY + rotX - yaw);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1) * speed; //Multiply by 1.7 to reduce speed

        leftFrontPower = leftFrontPower / denominator;
        rightFrontPower = rightFrontPower / denominator;
        leftBackPower = leftBackPower / denominator;
        rightBackPower = rightBackPower / denominator;*/


        double max;
        double lateral_1 = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double axial_1 = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);

        leftFrontPower = axial_1 + lateral_1 + yaw;
        rightFrontPower = axial_1 - lateral_1 - yaw;
        leftBackPower = axial_1 - lateral_1 + yaw;
        rightBackPower = axial_1 + lateral_1 - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }


        // Send calculated power to wheels.
        setPowerToWheels(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        //setSmoothPowerToWheels(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }


    public double smoothPower(double currentPower, double targetPower) {
            double sigma = 0.1;

        if (currentPower < targetPower) {
            return currentPower + sigma;
        } else if (currentPower > targetPower) {
            return currentPower - sigma;
        } else if (Math.abs(currentPower - targetPower) <= sigma) {
            return currentPower;
        }
        return currentPower;
    }

    public void setSmoothPowerToWheels(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {

        leftFrontPower = smoothPower(frontLeftDrive.getPower(), leftFrontPower);
        rightFrontPower = smoothPower(frontRightDrive.getPower(), rightFrontPower);
        leftBackPower = smoothPower(backLeftDrive.getPower(), leftBackPower);
        rightBackPower = smoothPower(backRightDrive.getPower(), rightBackPower);

        setPowerToWheels(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void setPowerToWheels(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
    }

    public void setPowerToWheels(double power) {
        setPowerToWheels(power, power, power, power);
    }

}

