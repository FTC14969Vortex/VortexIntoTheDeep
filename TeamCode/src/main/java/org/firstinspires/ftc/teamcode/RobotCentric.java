//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

@TeleOp(name = "RobotCentricSam", group = "TeleOp")

public class RobotCentric extends LinearOpMode {

    // The Motor objects
    private DcMotor FLMotor;
    private DcMotor BLMotor;
    private DcMotor FRMotor;
    private DcMotor BRMotor;
    // The IMU sensor object
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double coefficient = 0.1;


    @Override

    public void runOpMode() {
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


        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a) {

                odo.resetPosAndIMU();
            }
            odo.update();
            double botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
            telemetry.addLine("botHeading " + botHeading);
            double[] powerSetFB = calcPowerFB();
            double[] powerSetLR = calcPowerLR();
            double[] powerSetRot = calcPowerRot();

            FLMotor.setPower(powerSetFB[0] + powerSetLR[0] + powerSetRot[0]);
            telemetry.addData("Encoder Position FLMotor", FLMotor.getCurrentPosition());
            BLMotor.setPower(powerSetFB[1] + powerSetLR[1] + powerSetRot[1]);
            telemetry.addData("Encoder Position BLMotor", BLMotor.getCurrentPosition());
            FRMotor.setPower(powerSetFB[2] + powerSetLR[2] + powerSetRot[2]);
            telemetry.addData("Encoder Position FRMotor", FRMotor.getCurrentPosition());
            BRMotor.setPower(powerSetFB[3] + powerSetLR[3] + powerSetRot[3]);
            telemetry.addData("Encoder Position BRMotor", BRMotor.getCurrentPosition());
            telemetry.update();
            telemetry.addData("Encoder Position", BLMotor.getCurrentPosition());
            telemetry.update();

        }
    }

    private double[] calcPowerRot() {

        boolean ccRotation = gamepad1.dpad_left;
        boolean cRotation = gamepad1.dpad_right;
        odo.update();
        double botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
        telemetry.addLine("botHeading " + botHeading);
        double yaw = ccRotation ? 1 : 0;
        yaw = yaw == 0 & cRotation ? -1 : 0;
        double power = yaw * coefficient;
        double[] powerset = {-power, power, -power, power};
        return powerset;

    }

    private double[] calcPowerFB() {
        double power = -gamepad1.right_stick_y;
        double[] powerset = {power, power, power, power};
        return powerset;
    }

    private double[] calcPowerLR() {
        double power = gamepad1.right_stick_x;
        double[] powerset = {power, -power, -power, power};
        return powerset;
    }
}