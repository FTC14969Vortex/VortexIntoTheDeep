package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class chassis2 {
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    public GoBildaPinpointDriver odo; // Odometry + IMU

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private LinearOpMode opMode;
    private DriveMode driveMode;

    public chassis2(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public void init() {
        // Map drive motors
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        // Map odometry/imu
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // No offsets
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset IMU + Odo
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void resetIMU(){
        odo.resetPosAndIMU();
    }

    public void updateTelemetry() {
        odo.update();
        opMode.telemetry.addData("Drive Mode", driveMode);
        opMode.telemetry.addData("Front L/R", "%.2f, %.2f", leftFrontPower, rightFrontPower);
        opMode.telemetry.addData("Back  L/R", "%.2f, %.2f", leftBackPower, rightBackPower);
        opMode.telemetry.addData("Heading (rad)", "%.2f", odo.getHeading(AngleUnit.RADIANS));
        opMode.telemetry.addData("X (cm)", "%.2f", odo.getPosX(DistanceUnit.CM));
        opMode.telemetry.addData("Y (cm)", "%.2f", odo.getPosY(DistanceUnit.CM));
        opMode.telemetry.update();
    }

    // TeleOp drive control
    public void drive(double axial, double lateral, double yaw){
        double botHeading;
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = -odo.getHeading(AngleUnit.RADIANS);
        } else {
            botHeading = 0;
        }

        double lateral_1 = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double axial_1 = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);

        leftFrontPower = axial_1 + lateral_1 + yaw;
        rightFrontPower = axial_1 - lateral_1 - yaw;
        leftBackPower = axial_1 - lateral_1 + yaw;
        rightBackPower = axial_1 + lateral_1 - yaw;

        double max = JavaUtil.maxOfList(JavaUtil.createListWith(
                Math.abs(leftFrontPower), Math.abs(rightFrontPower),
                Math.abs(leftBackPower), Math.abs(rightBackPower)
        ));

        if (max > 1) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
    }

    // Move to a target Pose
    public void goToPosition(Pose2D targetPose, double maxPower, double timeoutSeconds) {
        double xTargetCM = targetPose.getX(DistanceUnit.CM);
        double yTargetCM = targetPose.getY(DistanceUnit.CM);
        double headingTargetRad = targetPose.getHeading(AngleUnit.RADIANS);

        final double POSITION_TOLERANCE_CM = 2.0;
        final double ANGLE_TOLERANCE_RAD = Math.toRadians(3);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opMode.opModeIsActive()) {
            odo.update();
            double currentHeading = -odo.getHeading(AngleUnit.RADIANS);

            double dx = xTargetCM - odo.getPosX(DistanceUnit.CM);
            double dy = yTargetCM - odo.getPosY(DistanceUnit.CM);
            double distance = Math.sqrt(dx * dx + dy * dy);

            double headingError = angleWrap(headingTargetRad - currentHeading);

            if (distance < POSITION_TOLERANCE_CM && Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                break;
            }

            double xPower = (dx > 0) ? 0.2 : -0.2;
            double yPower = (dy > 0) ? 0.2 : -0.2;
            double headingPower = (headingError > 0) ? 0.2 : -0.2;

            if (distance < POSITION_TOLERANCE_CM) {
                xPower = 0; yPower = 0;
            }
            if (Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                headingPower = 0;
            }

            drive(yPower, xPower, headingPower);

            opMode.telemetry.addData("Target X", "%.2f", xTargetCM);
            opMode.telemetry.addData("Target Y", "%.2f", yTargetCM);
            opMode.telemetry.addData("Current X", "%.2f", odo.getPosX(DistanceUnit.CM));
            opMode.telemetry.addData("Current Y", "%.2f", odo.getPosY(DistanceUnit.CM));
            opMode.telemetry.addData("Distance Error", "%.2f", distance);
            opMode.telemetry.addData("Heading Error (deg)", "%.2f", Math.toDegrees(headingError));
            opMode.telemetry.addData("Timer", runtime.seconds());
            opMode.telemetry.update();

            if (runtime.seconds() > timeoutSeconds) break;
        }

        drive(0,0,0);
    }

    // ✅ NEW: Turn in place to a target heading
    public void turnToAngle(double targetAngleRad, double timeoutSeconds) {
        final double ANGLE_TOLERANCE_RAD = Math.toRadians(2); // ~2 degrees

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opMode.opModeIsActive()) {
            odo.update();
            double currentHeading = -odo.getHeading(AngleUnit.RADIANS);
            double headingError = angleWrap(targetAngleRad - currentHeading);

            if (Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                break; // reached
            }

            double turnPower = (headingError > 0) ? 0.25 : -0.25;

            // Simple proportional option (instead of fixed power):
            // double turnPower = Math.max(-0.4, Math.min(0.4, headingError * 0.8));

            drive(0, 0, turnPower);

            opMode.telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngleRad));
            opMode.telemetry.addData("Current Angle (deg)", Math.toDegrees(currentHeading));
            opMode.telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
            opMode.telemetry.update();

            if (runtime.seconds() > timeoutSeconds) break;
        }

        drive(0,0,0);
    }

    public double angleWrap(double angle) {
        angle %= (2 * Math.PI);
        if (angle > Math.PI) angle -= (2 * Math.PI);
        if (angle < -Math.PI) angle += (2 * Math.PI);
        return angle;
    }
}
