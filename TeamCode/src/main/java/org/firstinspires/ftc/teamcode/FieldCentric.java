//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TeleOpCoachPeter", group = "TeleOp")

public class FieldCentric extends LinearOpMode {

    // The Motor objects
    private DcMotor FLMotor;
    private DcMotor BLMotor;
    private DcMotor FRMotor;
    private DcMotor BRMotor;
    // The IMU sensor object
    private IMU imu;

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
    @Override
    public void runOpMode() {
        double axial;
        double lateral;
        double yaw;

        FLMotor = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        BLMotor = hardwareMap.get(DcMotor.class, "backLeftDrive");
        FRMotor = hardwareMap.get(DcMotor.class, "frontRightDrive");
        BRMotor = hardwareMap.get(DcMotor.class, "backRightDrive");

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


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                imu.resetYaw();
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // stick Y direction: up is -, down is +
            // stick X direction: left is -, right is +
            axial = -gamepad1.left_stick_y; // Forward/Backward (inverted joystick for push forward = positive)
            lateral = gamepad1.left_stick_x; // Strafe left/right (positive is right, negation is left)
            yaw = gamepad1.right_stick_x; // Turn left/rigth (positive is clockwise, negative is counter-clockwise)

            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addLine("botHeading " + botHeading);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
            double rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
            rotX = rotX * 1.1; //counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1) * 1.7; //Multiply by 1.7 to reduce speed
            leftFrontPower = (rotY + rotX + yaw) / denominator;
            rightFrontPower = (rotY - rotX + yaw) / denominator;
            leftBackPower = (rotY - rotX - yaw) / denominator;
            rightBackPower = (rotY + rotX - yaw) / denominator;

            // Send calculated power to wheels.
            FLMotor.setPower(leftFrontPower);
            FRMotor.setPower(rightFrontPower);
            BLMotor.setPower(leftBackPower);
            BRMotor.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
            telemetry.update();

        }
    }
}