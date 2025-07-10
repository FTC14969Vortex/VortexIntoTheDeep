package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(-66.675, -95.25, DistanceUnit.MM);
        // TODO: Change Encoder Directions depending on your robot.
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

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

    public void resetIMU() {
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
    public void drive(double axial, double lateral, double yaw) {
        // If in field centric mode read botHeading from odo otherwise set botHeading equal to zero
        double botHeading;
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
        } else {
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
     *
     * @param targetPose     The target position.
     * @param maxPower       The maximum power to use.
     * @param timeoutSeconds The timeout in seconds.
     */
    public void goToPosition(Pose2D targetPose, double maxPower, double timeoutSeconds) {
        // Step 1: Get the target pose
        double xTargetCM = targetPose.getX(DistanceUnit.CM);
        double yTargetCM = targetPose.getY(DistanceUnit.CM);
        double headingTargetRad = targetPose.getHeading(AngleUnit.RADIANS);

        // Step 2: Define your "tolerances" – how close is close enough to stop.
        // Use the values from the example:
        final double POSITION_TOLERANCE_CM = 3.0;             // Stop if within 2cm
        final double ANGLE_TOLERANCE_RAD = Math.toRadians(3); // ~3 degrees

        // Step 3: Set up a timer to make sure your robot doesn't get stuck forever.
        // Initialize an ElapsedTime object and reset it.
        ElapsedTime timer = new ElapsedTime();
        // Step 4: Create the main control loop. The robot will keep doing these steps
        // until the OpMode is stopped or it reaches the target.
        // The loop should continue as long as the OpMode is active.
        while (opMode.opModeIsActive()) {
            // Step 4a: Update the odometry readings to get the robot's latest position.
            odo.update();
            // Step 4b: Get the robot's current X, Y, and Heading.
            // Remember that `odo.getPosX` and `odo.getPosY` can get values in CM.
            // Get the current heading in Radians.
            double currXInCM = odo.getPosX(DistanceUnit.CM);
            double currYInCM = odo.getPosY(DistanceUnit.CM);
            double currheadInRad = odo.getHeading(AngleUnit.RADIANS);
            // Step 4c: Calculate the "error" (how far off you are) for X, Y, and Heading.
            //   - Calculate `dx` (difference in X between target and current).
            //   - Calculate `dy` (difference in Y between target and current).
            //   - Calculate `distance` (straight-line distance to target using dx and dy).
            //   - Calculate `headingError` (difference in heading, remember to use `angleWrap`!).
            double dx = xTargetCM - currXInCM;
            double dy = yTargetCM - currYInCM;
            double distance = Math.sqrt(dx * dx + dy * dy);
            double headingError = headingTargetRad - currheadInRad;
            // Step 4d: Check if the robot is "close enough" to the target.
            // If the `distance` is less than `POSITION_TOLERANCE_CM` AND the absolute `headingError`
            // is less than `ANGLE_TOLERANCE_RAD`, then exit the loop.
            if (distance < POSITION_TOLERANCE_CM && headingError < ANGLE_TOLERANCE_RAD) {
                opMode.telemetry.addData("yibbbeee targesttt reeagged", "position error %f, heading error %f", distance, headingError);
                opMode.telemetry.update();
                break;
            }
            // Step 4e: Determine the motor powers using "Bang-Bang" control!
            // You'll set a fixed power (like 0.2 or -0.2) based on the sign of the error.
            //   - For `xPower`: If `dx` is positive, set `xPower` to 0.2 (move right); otherwise, set to -0.2 (move left).
            //   - For `yPower`: If `dy` is positive, set `yPower` to 0.2 (move forward); otherwise, set to -0.2 (move backward).
            //   - For `headingPower`: If `headingError` is positive, set `headingPower` to 0.2 (turn counter-clockwise);
            //     otherwise, set to -0.2 (turn clockwise).
            double axial = dy < 0 ? 0.2 : -0.2;
            double lateral = dx < 0 ? 0.2 : -0.2;
            double yaw = headingError < 0 ? 0.2 : -0.2;

            // Step 4f: Send these calculated powers to the robot's drive system.
            // Call the `drive` method, passing `yPower` as axial, `xPower` as lateral, and `headingPower` as yaw.
            drive(axial, lateral, yaw);
            // Step 4g: Display helpful information on the Driver Station (telemetry).
            // Use `opMode.telemetry.addData` to show:
            //   - Target X and Y.
            //   - Current X and Y.
            //   - Current distance error.
            //   - Current heading error (converted back to degrees for readability).
            // Call `opMode.telemetry.update()` to send the data.
            opMode.telemetry.addData("Target X and Y", "(%f, %f)", xTargetCM, yTargetCM);
            opMode.telemetry.addData("Current X and Y", "(%f, %f)", currXInCM, currYInCM);
            opMode.telemetry.addData("Current distance error", distance);
            opMode.telemetry.addData("Current heading error", Math.toDegrees(headingError));
            opMode.telemetry.update();
            //Step 4h: Checking Timeout.
            if (timer.seconds() > timeoutSeconds) {
                opMode.telemetry.addData("timed out", timer.seconds());
                opMode.telemetry.update();
                break;
            }
        }

        // Step 5: Once the loop finishes (either target reached or OpMode stopped),
        // stop the robot completely by calling the `drive` method with zero power for all directions.
        drive(0, 0, 0);
    }
}