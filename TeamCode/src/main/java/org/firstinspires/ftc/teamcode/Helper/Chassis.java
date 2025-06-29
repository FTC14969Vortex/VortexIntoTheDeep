package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

public class Chassis {
    // These variables store the power to apply to each motor.
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    // Reference to GoBilda's Pinpoint odometry driver
    public GoBildaPinpointDriver odo;

    // Drive motor references
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    // Link to LinearOpMode and drive mode selection
    private LinearOpMode opMode;
    private DriveMode driveMode;

    // PID control constants (adjust to tune performance)
    public double kP = 0.003;      // Proportional gain for distance
    public double kI = 0.0001;     // Integral gain for distance
    public double kD = 0.0002;     // Derivative gain for distance

    public double kP_angle = 0.8;  // Proportional gain for heading
    public double kI_angle = 0.0;  // Integral gain for heading
    public double kD_angle = 0.1;  // Derivative gain for heading

    // Enum to allow switching between field-centric and robot-centric driving
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    // Initialize hardware and reset position tracking
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        setDriveMode(DriveMode.FIELD_CENTRIC);

        // Link each motor to its config name
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set motor directions for correct movement
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Calibrate and zero the odometry system
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    // Change the drive mode (field-centric or robot-centric)
    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }

    // Immediately stop all motors
    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    // Calculates motor powers based on joystick input and drive mode
    public void drive(double axial, double lateral, double yaw, double speed) {
        // If field-centric, adjust input based on robot heading
        double botHeading = 0;
        if(driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = -odo.getHeading(AngleUnit.RADIANS);
            // This will likely change if the odometry unit is mounted differently.
        }

        // Apply heading rotation to axial/lateral values
        double lateral_1 = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double axial_1 = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);

        // Calculate raw motor powers
        leftFrontPower = speed*(axial_1 + lateral_1 + yaw);
        rightFrontPower = speed*(axial_1 - lateral_1 - yaw);
        leftBackPower = speed*(axial_1 - lateral_1 + yaw);
        rightBackPower = speed*(axial_1 + lateral_1 - yaw);

        // Normalize powers to stay within [-1, 1]
//        double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
//        if (max > 1) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }

        // Apply final power values to motors
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
    }

    // Returns the current pose from odometry in mm and radians
    public Pose2D getPoseEstimate() {
        double x = odo.getPosX(DistanceUnit.MM);
        double y = odo.getPosY(DistanceUnit.MM);
        double heading = odo.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }

    // Utility method to constrain values
    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Wrap angle between -PI and +PI
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    // Drives the robot to a specified pose using full PID control
    public void goToPosition(Pose2D targetPose, double driveSpeed, int timeoutMillis) {
        final double POSITION_TOLERANCE_MM = 10; // How close to the target to stop (distance)
        final double ANGLE_TOLERANCE_RAD = Math.toRadians(3); // How close to the angle to stop (rotation)
        ElapsedTime timer = new ElapsedTime();
        driveMode = DriveMode.FIELD_CENTRIC;
        // This method is used only for Auto. Auto is operated in field centric mode.

//        // PID variables for distance
//        double prevDistanceError = 0;
//        double integralDistance = 0;
//
//        // PID variables for heading
//        double prevAngleError = 0;
//        double integralAngle = 0;

        // Constants for proportional control.
        double kP_angle = 2.0/Math.PI; // When the angle is PI/2, drive at speed of 1.
        double kP_distance = 1.0/(6*25); // When the distnace is 25 mm, drive at speed of 1.

        while (opMode.opModeIsActive() && timer.milliseconds() < timeoutMillis) {
            Pose2D currentPose = getPoseEstimate();
            double dx = targetPose.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM);
            double dy = targetPose.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM);
            double distance = Math.hypot(dx, dy);

            double heading = currentPose.getHeading(AngleUnit.RADIANS);
            double angleToTarget = Math.atan2(dy, dx);
            double headingError = angleWrap(targetPose.getHeading(AngleUnit.RADIANS) - heading);
            double dx_speed = dx * kP_distance;
            double dy_speed = dy * kP_distance;
            double yaw_speed = headingError * kP_angle;



            if (dx < POSITION_TOLERANCE_MM && dy < POSITION_TOLERANCE_MM && Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                stop();
            }

            // Apply full PID for distance
//            integralDistance += distance;
//            double derivativeDistance = distance - prevDistanceError;
//            double speed = kP * distance + kI * integralDistance + kD * derivativeDistance;
//            prevDistanceError = distance;

//            // Apply full PID for heading
//            integralAngle += headingError;
//            double derivativeAngle = headingError - prevAngleError;
//            double turnSpeed = kP_angle * headingError + kI_angle * integralAngle + kD_angle * derivativeAngle;
//            prevAngleError = headingError;

            // Normalize direction vector and apply speed limit
//            double driveX = clip(relativeX / distance * speed, -maxPower, maxPower);
//            double driveY = clip(relativeY / distance * speed, -maxPower, maxPower);
//            double driveTurn = clip(turnSpeed, -maxPower, maxPower);

            // Drive robot based on calculated values
            drive(dy_speed, dx_speed, yaw_speed, driveSpeed);

            // Display debug values for tuning
            opMode.telemetry.addData("Distance Error (mm)", distance);
            opMode.telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
            opMode.telemetry.addData("dy_speed", dy_speed);
            opMode.telemetry.addData("dx_speed", dx_speed);
            opMode.telemetry.addData("yaw_speed", yaw_speed);
//            opMode.telemetry.addData("Integral Dist", integralDistance);
//            opMode.telemetry.addData("Derivative Dist", derivativeDistance);
//            opMode.telemetry.addData("Integral Angle", integralAngle);
//            opMode.telemetry.addData("Derivative Angle", derivativeAngle);
            opMode.telemetry.update();
        }

        // Stop motors once done
        stop();
    }

    // Follows a series of poses one after another using goToPosition()
    public void followPath(List<Pose2D> waypoints, double maxPower, int timeoutPerSegmentMillis) {
        for (Pose2D waypoint : waypoints) {
            goToPosition(waypoint, maxPower, timeoutPerSegmentMillis);
        }
    }

    // Turn robot in place to desired heading using proportional control
    public void turnToHeading(double targetHeadingRad, double maxTurnSpeed, int timeoutMillis) {
        final double ANGLE_TOLERANCE_RAD = Math.toRadians(2);
        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.milliseconds() < timeoutMillis) {
            double currentHeading = odo.getHeading(AngleUnit.RADIANS);
            double error = angleWrap(targetHeadingRad - currentHeading);

            if (Math.abs(error) < ANGLE_TOLERANCE_RAD) break;

            double turnPower = clip(error * 0.8, -maxTurnSpeed, maxTurnSpeed);
            drive(0, 0, turnPower, 0.5);
        }

        stop();
    }
}