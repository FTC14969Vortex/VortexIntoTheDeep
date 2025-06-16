//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;


public class Chassis {
    enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    private DriveMode dMode;
    private OpMode theOpMode;

    // The Motor objects
    private DcMotor FLMotor;
    private DcMotor BLMotor;
    private DcMotor FRMotor;
    private DcMotor BRMotor;
    // The IMU sensor object
    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private double leftFrontPower=0;
    private double leftBackPower=0;
    private double rightFrontPower=0;
    private double rightBackPower=0;

    private double rotX=0;
    private double rotY=0;
    private double botHeading=0;

    void init(OpMode opMode) {

        dMode = DriveMode.ROBOT_CENTRIC;
        theOpMode = opMode;

        FLMotor = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        BLMotor = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        FRMotor = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        BRMotor = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

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

    }

    // gamepad B
    void setDriveMode(DriveMode mode) {
        dMode = mode;        
    }  
    DriveMode getDriveMode() {
        return dMode;
    }


    // gamepad A
    void resetIMU() {
       odo.resetPosAndIMU();
    }

    // Print out
    // - Driving Mode
    // - Odometry computer's reading of (heading, x, y)
    void updateTelemetry() {
        theOpMode.telemetry.addLine("Driving mode " + (dMode==DriveMode.ROBOT_CENTRIC?
                    "ROBOT_CENTRIC":"FIELD_CENTRIC"));
        theOpMode.telemetry.addLine("botHeading " + botHeading);
        theOpMode.telemetry.addLine("rotX " + rotY);
        theOpMode.telemetry.addLine("rotY " + rotY);    

        theOpMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " +
            JavaUtil.formatNumber(rightFrontPower, 4, 2));
        theOpMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + 
            JavaUtil.formatNumber(rightBackPower, 4, 2));

        theOpMode.telemetry.update();
    }

    // TeleOp Mode Methods
    /**
     * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
     * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
     * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
     */
    void drive(double axial, double lateral, double yaw) {

            odo.update();
            if (dMode==Chassis.DriveMode.ROBOT_CENTRIC) {
                botHeading = 0;
            }
            else
                botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
            updateTelemetry();

            // Rotate the movement direction counter to the bot's rotation
            rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
            rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
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

    }
}
