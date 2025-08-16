package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
    private BNO055IMU imu;

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
        setDriveMode(DriveMode.ROBOT_CENTRIC);

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

        // Setup IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
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
            double distance = Math.hypot(dx, dy); // distance = sqrt(dx^2 + dy^2)

            double heading = currentPose.getHeading(AngleUnit.RADIANS);
            double angleToTarget = Math.atan2(dy, dx);
            double headingError = angleWrap(targetPose.getHeading(AngleUnit.RADIANS) - heading);
            double dx_speed = dx * kP_distance;
            double dy_speed = dy * kP_distance;
            double yaw_speed = headingError * kP_angle;



            if (dx < POSITION_TOLERANCE_MM && dy < POSITION_TOLERANCE_MM && Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                stop();
                break;
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

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    ElapsedTime elapsedTime = new ElapsedTime();

    public void moveByTime(Direction direction, double power, double seconds){
        if(!opMode.opModeIsActive()) return;


        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorWheelMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elapsedTime.reset();

        switch (direction){
            case FORWARD: setRobotPowerToWheels(power,power, power,power); break;
            case BACKWARD: setRobotPowerToWheels(-power,-power,-power,-power); break;
            case LEFT: setRobotPowerToWheels(-power, power, power, -power); break;
            case RIGHT: setRobotPowerToWheels(power, -power, -power, power); break;
        }

        while (opMode.opModeIsActive() && elapsedTime.seconds() < seconds){

        }

        setRobotPowerToWheels(0,0,0,0);
    }

    public void setMotorWheelMode(DcMotor.RunMode runMode){
        frontLeftDrive.setMode(runMode);
        frontRightDrive.setMode(runMode);
        backLeftDrive.setMode(runMode);
        backRightDrive.setMode(runMode);
    }

    public void setRobotPowerToWheels(double fl, double fr, double bl, double br){
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    public static double PI = 3.1415;

    //Encoder
    public static double COUNTS_PER_MOTOR_REV = 537.7;
    public static double DRIVE_GEAR_REDUCTION = 1.0;
    public static double WHEEL_DIAMETER_INCHES = 4.0;
    public static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * PI);

    public void moveWithEncoder(Direction direction, double power, double distanceInInches){
        if(!opMode.opModeIsActive()) return;

        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetTicks = (int) (distanceInInches * COUNTS_PER_INCH);

        elapsedTime.reset();

        switch (direction){
            case FORWARD: setTargetPositionToWheels(targetTicks,targetTicks, targetTicks,targetTicks); break;
            case BACKWARD: setTargetPositionToWheels(-targetTicks,-targetTicks,-targetTicks,-targetTicks); break;
            case LEFT: setTargetPositionToWheels(-targetTicks, targetTicks, targetTicks, -targetTicks); break;
            case RIGHT: setTargetPositionToWheels(targetTicks, -targetTicks, -targetTicks, targetTicks); break;
        }

        setMotorWheelMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = Math.abs(power);
        setRobotPowerToWheels(power,power, power,power);

        while (opMode.opModeIsActive() && isAnyWheelsBusy()){

        }

        setRobotPowerToWheels(0,0,0,0);
        setMotorWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static double P_DRIVE_COEFF = 0.05;
    public static final double minPower = 0.08;
    public void moveWithProportionalDeceleration(Direction direction, double maxPower, double distanceInches) {
        if (!opMode.opModeIsActive()) return;

        int targetTicks = (int) (distanceInches * COUNTS_PER_INCH);

        setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opMode.opModeIsActive()) {

            int remainingTicks = targetTicks - getAverageCurrentPositionAllWheels();
            if (remainingTicks <= 0) break;

            double progress = (double) remainingTicks / (double) targetTicks;
            double drivePower = minPower + (maxPower - minPower) * Math.pow(progress, 0.5);
            drivePower = Math.max(minPower, Math.min(drivePower, maxPower));


            //double calculatedPower = remainingTicks * P_DRIVE_COEFF;
            //double drivePower = Math.max(0.1, Math.min(Math.abs(calculatedPower), maxPower));


            switch (direction) {
                case FORWARD:
                    setRobotPowerToWheels(drivePower, drivePower, drivePower, drivePower);
                    break;
                case BACKWARD:
                    setRobotPowerToWheels(-drivePower, -drivePower, -drivePower, -drivePower);
                    break;
                case LEFT:
                    setRobotPowerToWheels(-drivePower, drivePower, drivePower, -drivePower);
                    break;
                case RIGHT:
                    setRobotPowerToWheels(drivePower, -drivePower, -drivePower, drivePower);
                    break;
            }
        }
        setRobotPowerToWheels(0,0,0,0);
    }


        //Gyroscope
    public static double HEADING_THRESHOLD = 1.0;
    public static double P_TURN_COEFF = 0.03;


    public void setTargetPositionToWheels(int fl, int fr, int bl, int br){
        frontLeftDrive.setTargetPosition(fl);
        frontRightDrive.setTargetPosition(fr);
        backLeftDrive.setTargetPosition(bl);
        backRightDrive.setTargetPosition(br);
    }

    public boolean isAnyWheelsBusy(){
        return frontLeftDrive.isBusy() ||
                frontRightDrive.isBusy() ||
                backLeftDrive.isBusy() ||
                backRightDrive.isBusy();
    }
    public void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        frontLeftDrive.setZeroPowerBehavior(behavior);
        frontRightDrive.setZeroPowerBehavior(behavior);
        backLeftDrive.setZeroPowerBehavior(behavior);
        backRightDrive.setZeroPowerBehavior(behavior);
    }

    public int getAverageCurrentPositionAllWheels(){

     int fl = Math.abs(frontLeftDrive.getCurrentPosition());
     int bl = Math.abs(backLeftDrive.getCurrentPosition());
     int fr = Math.abs(frontRightDrive.getCurrentPosition());
     int br = Math.abs(backRightDrive.getCurrentPosition());

     int averageCurrentPosition = (fl + bl + fr + br)/4;
     return averageCurrentPosition;

    }
    private double wrap180(double deg) {
        double x = (deg + 180.0) % 360.0;
        if (x < 0) x += 360.0;
        return x - 180.0;
    }
    private double getHeadingDeg() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, // angles relative to starting orientation
                AxesOrder.ZYX,           // order Z (yaw), Y (pitch), X (roll)
                AngleUnit.DEGREES);

        // yaw is the "firstAngle" in this AxesOrder
        double heading = angles.firstAngle;
        return wrap180(heading);
    }
    public void moveWithProportionalDecelerationAndHeading(
            Direction direction, double maxPower, double distanceInches, Double holdHeadingDeg) {

        double MIN_POWER       = 0.08;  // just above stall for your drivetrain
         double KP_HEADING      = 0.012; // start here; tune on the field
         double KI_HEADING      = 0.000; // optional (keep 0 to start)
         double KD_HEADING      = 0.000; // optional (keep 0 to start)
         double MAX_YAW_CORR    = 0.25;  // cap on turn correction (0..1)
         int    STOP_TOL_TICKS  = 20;    // ~0.4 in @ 45 cpi


        if (!opMode.opModeIsActive()) return;

        // --- Prep encoders ---
        int targetTicks = (int) Math.round(Math.abs(distanceInches) * COUNTS_PER_INCH);
        setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Prep IMU ---
        // Assuming "imu" is com.qualcomm.hardware.bosch.BNO055IMU or the newer IMU interface and already initialized.
        // Choose degrees for angle unit when you init IMU elsewhere.
        double yaw0 = getHeadingDeg(); // your helper that reads yaw in degrees
        double headingSetpoint = (holdHeadingDeg != null) ? holdHeadingDeg : yaw0;

        // Heading PID state
        double lastErr = 0, errI = 0;
        long   lastT   = System.nanoTime();

        while (opMode.opModeIsActive()) {

            int remaining = targetTicks - getAverageCurrentPositionAllWheels();
            if (remaining <= STOP_TOL_TICKS) break;

            // Linear taper (1 -> 0)
            double progress = Math.max(0.0, Math.min(1.0, (double) remaining / (double) targetTicks));
            // Optional easing for earlier slowdown (gamma<1 decelerates earlier)
            double gamma = 0.7;
            double tapered = Math.pow(progress, gamma);

            double drivePower = MIN_POWER + (maxPower - MIN_POWER) * tapered;
            drivePower = Range.clip(drivePower, MIN_POWER, maxPower);

            // --- Heading hold (PID) ---
            double yaw = getHeadingDeg(); // current yaw in degrees
            double err = wrap180(headingSetpoint - yaw);

            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastT) / 1e9); // seconds
            lastT = now;

            // PI(D)
            errI += err * dt;
            // simple anti-windup
            errI = Range.clip(errI, -50.0, 50.0);

            double errD = (err - lastErr) / dt;
            lastErr = err;

            double turnCorr = KP_HEADING * err + KI_HEADING * errI + KD_HEADING * errD;
            turnCorr = Range.clip(turnCorr, -MAX_YAW_CORR, MAX_YAW_CORR);

            // --- Command mix ---
            // axial: forward/back; lateral: strafe; yaw: heading correction
            double axial = 0, lateral = 0;
            switch (direction) {
                case FORWARD:  axial =  drivePower; break;
                case BACKWARD: axial = -drivePower; break;
                case LEFT:     lateral =  drivePower; break;
                case RIGHT:    lateral = -drivePower; break;
            }
            double yawCmd = turnCorr;

            // mecanum mix
            double flPower = axial + lateral + yawCmd;
            double frPower = axial - lateral - yawCmd;
            double blPower = axial - lateral + yawCmd;
            double brPower = axial + lateral - yawCmd;

            // Normalize if any exceeds 1.0
            double maxAbs = Math.max(1.0,
                    Math.max(Math.abs(flPower),
                            Math.max(Math.abs(frPower),
                                    Math.max(Math.abs(blPower), Math.abs(brPower)))));
            flPower /= maxAbs; frPower /= maxAbs; blPower /= maxAbs; brPower /= maxAbs;

            // Scale to preserve the intended drive magnitude (keep <= maxPower)
            flPower *= maxPower; frPower *= maxPower; blPower *= maxPower; brPower *= maxPower;

            // Ensure we don't drop below MIN_POWER along the commanded axis (helps overcome static friction)
            // but allow the heading correction to modulate around it.
            // Only enforce MIN_POWER on the dominant drive component:
            if (direction == Direction.FORWARD || direction == Direction.BACKWARD) {
                double sign = Math.signum(axial);
                flPower = sign * Math.max(MIN_POWER, Math.abs(flPower));
                frPower = sign * Math.max(MIN_POWER, Math.abs(frPower));
                blPower = sign * Math.max(MIN_POWER, Math.abs(blPower));
                brPower = sign * Math.max(MIN_POWER, Math.abs(brPower));
            } else {
                double sign = Math.signum(lateral);
                // For strafes, friction is higher—MIN_POWER helps a lot
                flPower = (Math.signum(flPower) == 0 ? sign : Math.signum(flPower)) * Math.max(MIN_POWER, Math.abs(flPower));
                frPower = (Math.signum(frPower) == 0 ? sign : Math.signum(frPower)) * Math.max(MIN_POWER, Math.abs(frPower));
                blPower = (Math.signum(blPower) == 0 ? sign : Math.signum(blPower)) * Math.max(MIN_POWER, Math.abs(blPower));
                brPower = (Math.signum(brPower) == 0 ? sign : Math.signum(brPower)) * Math.max(MIN_POWER, Math.abs(brPower));
            }

            // Apply powers
            setRobotPowerToWheels( flPower, frPower, blPower, brPower);

        }

        // Stop hard
        setRobotPowerToWheels( 0, 0, 0, 0);
    }

}