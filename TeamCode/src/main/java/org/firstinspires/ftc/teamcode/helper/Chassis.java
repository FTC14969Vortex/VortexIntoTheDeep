package org.firstinspires.ftc.teamcode.helper;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    private OpMode opMode;
    private DriveMode driveMode;

    public enum DriveMode {

        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public void init(OpMode opMode) {

        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightBack");
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

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
            botHeading = -odo.getHeading(AngleUnit.RADIANS);// Get the robot's heading in radians
            
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
}