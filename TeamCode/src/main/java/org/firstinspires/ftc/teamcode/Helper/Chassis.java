package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

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

    private LinearOpMode opMode;
    private DriveMode driveMode;

    public enum DriveMode {

        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public void init(LinearOpMode opMode) {

        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(-66.675, -95.25, DistanceUnit.MM);
        // TODO: Change Encoder Directions depending on your robot.
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

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
    public void resetIMU(){
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
    public void drive(double axial, double lateral, double yaw){
        // If in field centric mode read botHeading from odo otherwise set botHeading equal to zero
        double botHeading;
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
        }
        else {
            botHeading = 0;
        }

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
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);

    }

    /**
     * Moves the robot to the target position.
     * @param targetPose The target position.
     * @param maxPower The maximum power to use.
     * @param timeoutSeconds The timeout in seconds.
     */
    public void goToPosition(Pose2D targetPose, double maxPower, double timeoutSeconds) {
        // Step 1: Get the target pose
        double xTargetCM = targetPose.getX(DistanceUnit.CM);
        double yTargetCM = targetPose.getY(DistanceUnit.CM);
        double headingTargetRad = targetPose.getHeading(AngleUnit.RADIANS);

        // Step 2: Define your "tolerances"
        final double POSITION_TOLERANCE_CM = 3.0;             // Stop if within 3cm
        final double ANGLE_TOLERANCE_RAD = Math.toRadians(3); // ~3 degrees

<<<<<<< Updated upstream
        // Step 3: Set up a timer
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        timer.startTime();

        // Step 4: Main control loop
        while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
            // Step 4a: Update the odometry readings
            odo.update();

            // Step 4b: Get current position
            double currentX = odo.getPosX(DistanceUnit.CM);
            double currentY = odo.getPosY(DistanceUnit.CM);
            double currentHeading = -odo.getHeading(AngleUnit.RADIANS);

            // Step 4c: Calculate errors
            double dx = xTargetCM - currentX;
            double dy = yTargetCM - currentY;
            double distance = Math.hypot(dx, dy);
            double headingError = normalizeAngle(headingTargetRad - currentHeading);

            // Step 4d: Check if close enough to target
=======
        // Step 3: Set up a timer to make sure your robot doesn't get stuck forever.
        // Initialize an ElapsedTime object and reset it.
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Step 4: Create the main control loop. The robot will keep doing these steps
        // until the OpMode is stopped or it reaches the target.
        // The loop should continue as long as the OpMode is active.
        while (opMode.opModeIsActive()) {
            // Step 4a: Update the odometry readings to get the robot's latest position.
            odo.update();
            // Step 4b: Get the robot's current X, Y, and Heading.
            // Remember that `odo.getPosX` and `odo.getPosY` can get values in CM.
            // Get the current heading in Radians.
            double currentX = odo.getPosX(DistanceUnit.CM);
            double currentY = odo.getPosY(DistanceUnit.CM);
            double currentHeading = odo.getHeading(AngleUnit.RADIANS);

            // Step 4c: Calculate the "error" (how far off you are) for X, Y, and Heading.
            //   - Calculate `dx` (difference in X between target and current).
            //   - Calculate `dy` (difference in Y between target and current).
            //   - Calculate `distance` (straight-line distance to target using dx and dy).
            //   - Calculate `headingError` (difference in heading, remember to use `angleWrap`!).
            double dx = xTargetCM - currentX;
            double dy = yTargetCM - currentY;
            double distance = Math.hypot(dx, dy);
            double headingError = angleWrap(headingTargetRad - currentHeading);

            // Step 4d: Check if the robot is "close enough" to the target.
            // If the `distance` is less than `POSITION_TOLERANCE_CM` AND the absolute `headingError`
            // is less than `ANGLE_TOLERANCE_RAD`, then exit the loop.
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            if (distance < POSITION_TOLERANCE_CM && Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                break;
            }

<<<<<<< Updated upstream
            // Step 4e: motor powers
            double xPower;
            double yPower;
            double headingPower;
            if (dx > 0) {
                xPower = 0.2;
            } else {
                xPower = -0.2;
            }
            if (dy > 0) {
                yPower = 0.2;
            } else {
                yPower = -0.2;
            }
            if (headingError > 0) {
                headingPower = 0.2;
            } else {
                headingPower = -0.2;
            }

            if (Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                headingPower = 0;
            }
            if (distance < POSITION_TOLERANCE_CM) {
                xPower = 0;
                yPower = 0;
            }
            // Step 4f: Drive the robot
            drive(yPower, xPower, headingPower);
=======
            // Step 4e: Determine the motor powers using "Bang-Bang" control!
            // You'll set a fixed power (like 0.2 or -0.2) based on the sign of the error.
            //   - For `xPower`: If `dx` is positive, set `xPower` to 0.2 (move right); otherwise, set to -0.2 (move left).
            //   - For `yPower`: If `dy` is positive, set `yPower` to 0.2 (move forward); otherwise, set to -0.2 (move backward).
            //   - For `headingPower`: If `headingError` is positive, set `headingPower` to 0.2 (turn counter-clockwise);
            //     otherwise, set to -0.2 (turn clockwise).
            double targetAngle = Math.atan2(dy, dx);
            double rotatedX = dx * Math.cos(-currentHeading) - dy * Math.sin(-currentHeading);
            double rotatedY = dx * Math.sin(-currentHeading) + dy * Math.cos(-currentHeading);
            double xPower = rotatedX / (Math.abs(rotatedX) + Math.abs(rotatedY));
            double yPower = rotatedY / (Math.abs(rotatedX) + Math.abs(rotatedY));
            xPower *= maxPower;
            yPower *= maxPower;

            // Step 4f: Send these calculated powers to the robot's drive system.
            // Call the `drive` method, passing `yPower` as axial, `xPower` as lateral, and `headingPower` as yaw.
            drive(yPower, xPower, headingPower);
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
=======
=======
=======
=======
=======

>>>>>>> Stashed changes

>>>>>>> Stashed changes

>>>>>>> Stashed changes

>>>>>>> Stashed changes

>>>>>>> Stashed changes

>>>>>>> Stashed changes

>>>>>>> Stashed changes

            // Step 4g: Display telemetry
            opMode.telemetry.addData("Target X", xTargetCM);
            opMode.telemetry.addData("Target Y", yTargetCM);
            opMode.telemetry.addData("Current X", currentX);
            opMode.telemetry.addData("Current Y", currentY);
            opMode.telemetry.addData("Distance Error", distance);
            opMode.telemetry.addData("Heading Error", headingError);
            opMode.telemetry.addData("Target Heading", headingTargetRad);
            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.update();
        }

        // Step 5: Stop the robot
        drive(0, 0, 0);
    }

    /**
     * Normalizes the angle to be within -π to π.
     * @param angle Input angle in radians
     * @return Normalized angle in radians
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
