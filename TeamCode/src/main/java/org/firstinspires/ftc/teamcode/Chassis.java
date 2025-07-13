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
    public AutonomousDrive myTeleop;
    public DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    double botHeading;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    public enum opsMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    opsMode currentMode = opsMode.ROBOT_CENTRIC;
    final double Tolerence_distance_cm = 2;
    final double Tolerence_heading_rad = Math.toRadians(3);
    ElapsedTime timer= new ElapsedTime();

    public Chassis(AutonomousDrive teleOp) {
        myTeleop = teleOp;

        odo = myTeleop.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        frontLeftDrive = myTeleop.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = myTeleop.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = myTeleop.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = myTeleop.hardwareMap.get(DcMotor.class, "backRightDrive");
    }

    public void goToPosition(double xTargetCM, double yTargetCM, double headingTargetDeg, double maxPower, double timeoutSeconds) {
        this.goToPosition(new Pose2D(DistanceUnit.CM, xTargetCM, yTargetCM, AngleUnit.DEGREES, headingTargetDeg), maxPower, timeoutSeconds);
    }

    private void goToPosition(Pose2D targetPose, double maxPower, double timeoutSeconds) {
        // step 1
        double xTargetCM = targetPose.getX(DistanceUnit.CM);
        double yTargetCM = targetPose.getY(DistanceUnit.CM);
        double headingTargetRad = targetPose.getHeading(AngleUnit.RADIANS);

        // Step 2

        // Step 3
        timer.reset();
        timer.startTime();

        // Step 4
        while (myTeleop.opModeIsActive()) {
            if (timer.seconds() > timeoutSeconds) {
                myTeleop.telemetry.addData("Path", "Timed out");
                myTeleop.telemetry.update();
                break;
            }

            // Step 4a
            odo.update();

            // Step 4b
            double xCurrentCM = odo.getPosX(DistanceUnit.CM);
            double yCurrentCM = odo.getPosY(DistanceUnit.CM);
            double headingCurrentRad = odo.getHeading(AngleUnit.RADIANS);

            // Step 4c
            double dx = xTargetCM - xCurrentCM;
            double dy = yTargetCM - yCurrentCM;
            double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            double headingError = normalizeAngle(headingTargetRad - headingCurrentRad);

            // Step 4d
            if ((distance < Tolerence_distance_cm) && (Math.abs(headingError) < Tolerence_heading_rad)) {
                break;
            }

            //Step 4e
            double xPower = 0;
            double yPower= 0;
            double headingPower= 0;
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

            // Step 4f
            this.drive(xPower, yPower, headingPower);

            // Step 4g
            myTeleop.telemetry.addData("Target X", xTargetCM);
            myTeleop.telemetry.addData("Target Y", yTargetCM);
            myTeleop.telemetry.addData( "Current X", xCurrentCM);
            myTeleop.telemetry.addData( "Current Y", yCurrentCM);
            myTeleop.telemetry.addData( "Distance Error", distance);
            myTeleop.telemetry.addData( "Heading Error", Math.toDegrees(headingError));
            myTeleop.telemetry.update();
        }

        // Step 5
        drive(0,0,0);
        myTeleop.telemetry.addData("Path", "Complete");
        myTeleop.telemetry.update();


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
