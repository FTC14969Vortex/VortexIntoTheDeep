//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

@TeleOp(name = "FieldCentricSam", group = "TeleOp")

public class FieldCentric extends LinearOpMode {

    // The Motor objects
    private DcMotor FLMotor;
    private DcMotor BLMotor;
    private DcMotor FRMotor;
    private DcMotor BRMotor;
    // The IMU sensor object
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    @Override

    public void runOpMode() {
        double axial;
        double lateral;
        double yaw;

        FLMotor = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        BLMotor = hardwareMap.get(DcMotor.class, "backLeftDrive");
        FRMotor = hardwareMap.get(DcMotor.class, "frontRightDrive");
        BRMotor = hardwareMap.get(DcMotor.class, "backRightDrive");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

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

        waitForStart();

        while (opModeIsActive()) {
            double power;
            if (gamepad1.a) {
                odo.resetPosAndIMU();
            }
            power = gamepad1.right_stick_x;
            FLMotor.setPower(power);

            telemetry.addData("Encoder Position", FLMotor.getCurrentPosition());
            BLMotor.setPower(power);

            telemetry.addData("Encoder Position", BLMotor.getCurrentPosition());
            FRMotor.setPower(power);

            telemetry.addData("Encoder Position", FRMotor.getCurrentPosition());
            BRMotor.setPower(power);

            telemetry.addData("Encoder Position", BRMotor.getCurrentPosition());
            telemetry.update();
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // stick Y direction: up is -, down is +
            // stick X direction: left is -, right is +
            axial = -gamepad1.left_stick_y; // Forward/Backward (inverted joystick for push forward = positive)
            lateral = gamepad1.left_stick_x; // Strafe left/right (positive is right, negation is left)
            yaw = gamepad1.right_stick_x; // Turn left/rigth (positive is clockwise, negative is counter-clockwise)

            telemetry.addData("Encoder Position", BLMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}