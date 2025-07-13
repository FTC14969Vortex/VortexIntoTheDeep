//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Chassis {
    public AutonomousDrive myDrive;
    public DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    double botHeading;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    public enum opsMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    opsMode currentMode = opsMode.ROBOT_CENTRIC;

    //** Autonomous Tolerance
    final double TOLERANCE_DISTANCE_CM = 2;
    final double TOLERANCE_HEADING_RAD = Math.toRadians(3);
    ElapsedTime timer = new ElapsedTime();

    public Chassis(AutonomousDrive objDrive) {

        myDrive = objDrive;
        odo = myDrive.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        frontLeftDrive = myDrive.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = myDrive.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = myDrive.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = myDrive.hardwareMap.get(DcMotor.class, "backRightDrive");
    }


    public void goToPosition(double xTargetCM, double yTargetCM, double headingTargetDeg, double maxPower, double timeoutSeconds) {
        this.goToPosition(new Pose2D(DistanceUnit.CM, xTargetCM, yTargetCM, AngleUnit.DEGREES, headingTargetDeg), maxPower, timeoutSeconds);
    }

    private void goToPosition(Pose2D targetPose, double maxPower, double timeoutSeconds) {

        //** Step 1: Set target destination location
        double xTargetCM = targetPose.getX(DistanceUnit.CM);
        double yTargetCM = targetPose.getY(DistanceUnit.CM);
        double headingTargetRad = targetPose.getHeading(AngleUnit.RADIANS);

        //** Step 2: Set the tolerance for position and heading degree
        // set in the global class level

        //** Step 3: Setup timer (watchdog timer) in case running in an endless loop
        timer.reset();
        timer.startTime();


        //** Step 4: Loop as long as the OpMode is active or timeout
        while (myDrive.opModeIsActive()) {
            if (timer.seconds() > timeoutSeconds) {
                myDrive.telemetry.addData("Path", "Timed out");
                myDrive.telemetry.update();
                break;
            }

            //** Step 4a: Update odometry readings to get the robot's latest position
            odo.update();

            //** Step 4b: Get robot's current position and heading angle
            double xCurrentCM = odo.getPosX(DistanceUnit.CM);
            double yCurrentCM = odo.getPosY(DistanceUnit.CM);
            double headingCurrentRad = odo.getHeading(AngleUnit.RADIANS);

            // Step 4c: Calculate delta to target in x, y, and angle
            //   - Calculate `dx` (difference in X between target and current).
            double dx = xTargetCM - xCurrentCM;
            //   - Calculate `dy` (difference in Y between target and current).
            double dy = yTargetCM - yCurrentCM;
            //   - Calculate `distance` (straight-line distance to target using dx and dy).
            double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            //   - Calculate `headingError` (difference in heading, remember to use `angleWrap`!).
            double headingError = normalizeAngle(headingTargetRad - headingCurrentRad);

            // Step 4d: Check if the robot reaches the target location. Exist if so
            if ((distance < TOLERANCE_DISTANCE_CM) && (Math.abs(headingError) < TOLERANCE_HEADING_RAD)) {
                break;
            }

            // Step 4e: Determine the motor powers using "Bang-Bang" control!
            // You'll set a fixed power (like 0.2 or -0.2) based on the sign of the error.
            //   - For `xPower`: If `dx` is positive, set `xPower` to 0.2 (move right); otherwise, set to -0.2 (move left).
            //   - For `yPower`: If `dy` is positive, set `yPower` to 0.2 (move forward); otherwise, set to -0.2 (move backward).
            //   - For `headingPower`: If `headingError` is positive, set `headingPower` to 0.2 (turn counter-clockwise);
            //     otherwise, set to -0.2 (turn clockwise).
            double xPower = 0;
            double yPower = 0;
            double headingPower = 0;
            if (dx > 0) {
                xPower = maxPower;
            } else {
                xPower = -maxPower;
            }
            if (dy > 0) {
                yPower = maxPower;
            } else {
                yPower = -maxPower;
            }
            if (headingError > 0) {
                headingPower = maxPower;
            } else {
                headingPower = -maxPower;
            }

            // Step 4f: Send these calculated powers to the robot's drive system.
            // Call the `drive` method, passing `yPower` as axial, `xPower` as lateral, and `headingPower` as yaw.
            this.drive(yPower, xPower, headingPower);

            // Step 4g: Display helpful information on the Driver Station (telemetry).
            // Use `opMode.telemetry.addData` to show:
            //   - Target X and Y.
            //   - Current X and Y.
            //   - Current distance error.
            //   - Current heading error (converted back to degrees for readability).
            // Call `opMode.telemetry.update()` to send the data.
            myDrive.telemetry.addData("Target X", xTargetCM);
            myDrive.telemetry.addData("Target Y", yTargetCM);
            myDrive.telemetry.addData("Current X", xCurrentCM);
            myDrive.telemetry.addData("Current Y", yCurrentCM);
            myDrive.telemetry.addData("Distance Error", distance);
            myDrive.telemetry.addData("Heading Error", Math.toDegrees(headingError));
            myDrive.telemetry.update();

        }

        // Step 5: stop the robot completely by calling the `drive` method with zero power for all directions.
        drive(0, 0, 0);
        myDrive.telemetry.addData("Path", "Complete");
        myDrive.telemetry.update();

    }

    public void setRobotCentric() {
        currentMode = opsMode.ROBOT_CENTRIC;
    }

    public void setFieldCentric() {
        currentMode = opsMode.FIELD_CENTRIC;
    }

    public void init() {

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void resetIMU() {
        odo.resetPosAndIMU();
    }
    public void drive(double axial, double lateral, double yaw) {

        if (currentMode == opsMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = -odo.getHeading(AngleUnit.RADIANS); // Get the robot's heading in radians
            double rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
            double rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
            rotX = rotX * 1.1; //counteract imperfect strafing
            axial = rotY;
            lateral = rotX;
        }

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = (axial - lateral) - yaw;
        double leftBackPower = (axial - lateral) + yaw;
        double rightBackPower = (axial + lateral) - yaw;

        double speed = 1.7;
        double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower))) * speed;

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

