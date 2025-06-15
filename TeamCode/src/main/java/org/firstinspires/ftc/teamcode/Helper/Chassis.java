package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class Chassis {
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

//    ElapsedTime runtime;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer


    private OpMode opMode;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    DriveMode driveMode;

    public void init(OpMode opMode) {
        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
 //       runtime = new ElapsedTime();
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }
    public void setDriveMode(DriveMode mode) { // gamepad B

        driveMode = mode;
    }
    public void toggleDriveMode() {
        if (driveMode == DriveMode.ROBOT_CENTRIC) {
            driveMode = DriveMode.FIELD_CENTRIC;
        } else {
            driveMode = DriveMode.ROBOT_CENTRIC;
        }
    }
    public void resetIMU() {  // gamepad A
        odo.resetPosAndIMU();
    }
    // Print out
    // - Driving Mode
    // - Odometry computer's reading of (heading, x, y)
    void updateTelemetry() {
//        opMode.telemetry.addData("Status", "Run Time: " + runtime);
        opMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
        opMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
        opMode.telemetry.update();
    }
    // TeleOp Mode Methods
    /**
     * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
     * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
     * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
     */
    public void drive(double axial, double lateral, double yaw) {

        if (driveMode == DriveMode.ROBOT_CENTRIC) {
            double max;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = (axial - lateral) - yaw;
            leftBackPower = (axial - lateral) + yaw;
            rightBackPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1) {
                leftFrontPower = leftFrontPower / max;
                rightFrontPower = rightFrontPower / max;
                leftBackPower = leftBackPower / max;
                rightBackPower = rightBackPower / max;
            }

        } else {
            // field centric
            odo.update();
            double botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
            opMode.telemetry.addLine("botHeading " + botHeading);

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
        }

        // Send calculated power to wheels.
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        updateTelemetry();
    }
}
