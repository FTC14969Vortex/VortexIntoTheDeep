package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

@TeleOp(name = "ChassisHam", group = "TeleOp")
public class Chassis extends LinearOpMode {
    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

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
    double speed = 1.7;
    double rotationScalingFactor = 1.1;


    @Override
    public void runOpMode() throws InterruptedException {

        double axial;
        double lateral;
        double yaw;

        FLMotor = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        BLMotor = hardwareMap.get(DcMotor.class, "backLeftDrive");
        FRMotor = hardwareMap.get(DcMotor.class, "frontRightDrive");
        BRMotor = hardwareMap.get(DcMotor.class, "backRightDrive");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

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
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                odo.resetPosAndIMU();
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // stick Y direction: up is -, down is +
            // stick X direction: left is -, right is +
            axial = -gamepad1.left_stick_y; // Forward/Backward (inverted joystick for push forward = positive)
            lateral = gamepad1.left_stick_x; // Strafe left/right (positive is right, negation is left)
            yaw = gamepad1.right_stick_x; // Turn left/rigth (positive is clockwise, negative is counter-clockwise)

            double[] powerset = calcFieldCentricPower(lateral, axial, yaw);

            // Send calculated power to wheels.
            FLMotor.setPower(powerset[0]);
            FRMotor.setPower(powerset[1]);
            BLMotor.setPower(powerset[2]);
            BRMotor.setPower(powerset[3]);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
            telemetry.update();

        }

    }

    private double @NonNull [] calcFieldCentricPower(double lateral, double axial, double yaw) {
        odo.update();
        double botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
        telemetry.addLine("botHeading " + botHeading);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
        rotX = rotX * rotationScalingFactor; //counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1) * speed; //Multiply by 1.7 to reduce speed
        double[] powerset = {(rotY + rotX + yaw) / denominator, (rotY - rotX - yaw) / denominator, (rotY - rotX + yaw) / denominator, (rotY + rotX - yaw) / denominator};
        return powerset;
    }

    enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    void init(OpMode opMode) {

    }

    void setDriveMode(DriveMode mode) {

    }
    // gamepad B

    void resetIMU() {

    }
    // gamepad A

    // Print out
    // - Driving Mode
    // - Odometry computer's reading of (heading, x, y)
    void updateTelemetry() {

    }
    // TeleOp Mode Methods

    /**
     * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
     * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
     * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
     */
    void drive(double axial, double lateral, double yaw) {

    }
}