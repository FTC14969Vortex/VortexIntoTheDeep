package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


// Assuming you have an Odometry class or similar
// import org.firstinspires.ftc.teamcode.Odometry; // Replace with your actual Odometry class

public class Chassis {

    // --- You'll need to have these members in your Chassis class ---
    private LinearOpMode opMode;
    // Assuming you have an Odometry object.
    // Replace 'Odometry' with the actual class name and ensure it's initialized.
    private Odometry odo;
    private ElapsedTime runtime = new ElapsedTime(); // For timeouts if you add them

    // Example DriveMode enum (if you use it elsewhere)
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    private DriveMode currentDriveMode = DriveMode.ROBOT_CENTRIC; // Example initialization

    // Constructor or init method to set up opMode and odo
    public Chassis() {
        // Default constructor
    }

    public void init(LinearOpMode opModeInstance, Odometry odometryInstance) {
        this.opMode = opModeInstance;
        this.odo = odometryInstance;
        // Initialize motors, sensors, etc.
        // e.g., leftFrontDrive = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        opMode.telemetry.addData("Chassis", "Initialized");
        opMode.telemetry.update();
    }

    // Your existing drive method (or a similar one)
    public void drive(double axial, double lateral, double yaw) {
        // This is where you would calculate and set power to your robot's motors
        // based on the desired axial (forward/backward), lateral (strafe),
        // and yaw (turning) inputs.
        // For example:
        // double leftFrontPower = axial + lateral + yaw;
        // double rightFrontPower = axial - lateral - yaw;
        // double leftBackPower = axial - lateral + yaw;
        // double rightBackPower = axial + lateral - yaw;
        //
        // // Normalize powers if they exceed 1.0
        // double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        // max = Math.max(max, Math.abs(leftBackPower));
        // max = Math.max(max, Math.abs(rightBackPower));
        //
        // if (max > 1.0) {
        //     leftFrontPower /= max;
        //     rightFrontPower /= max;
        //     leftBackPower /= max;
        //     rightBackPower /= max;
        // }
        //
        // // Assuming you have motor objects like leftFrontDrive, etc.
        // leftFrontDrive.setPower(leftFrontPower);
        // rightFrontDrive.setPower(rightFrontPower);
        // leftBackDrive.setPower(leftBackPower);
        // rightBackDrive.setPower(rightBackPower);

        if (opMode != null) {
            opMode.telemetry.addData("Chassis Drive Cmd", "Ax: %.2f, Lat: %.2f, Yaw: %.2f", axial, lateral, yaw);
            // opMode.telemetry.update(); // Update telemetry in the main loop instead for less spam
        }
    }

    /**
     * Helper function to ensure an angle is within the range of -PI to PI radians.
     * @param angle The angle in radians.
     * @return The angle wrapped to the range [-PI, PI].
     */
    private double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    // Method signature provided for you. Don't change this line.
    public void goToPosition(double xTargetCM, double yTargetCM, double headingTargetDeg, double maxPower) {
        if (opMode == null || odo == null) {
            // Or throw an IllegalStateException
            if (opMode != null) opMode.telemetry.addData("Error", "Chassis not fully initialized (opMode or odo is null)");
            if (opMode != null) opMode.telemetry.update();
            return;
        }

        // Step 1: Convert the target heading from degrees to radians.
        double headingTargetRad = Math.toRadians(headingTargetDeg);

        // Step 2: Define your "tolerances" – how close is close enough to stop.
        final double POSITION_TOLERANCE_CM = 2.0;
        final double ANGLE_TOLERANCE_RAD = Math.toRadians(3.0); // 3 degrees in radians

        // Step 3: Set up a timer to make sure your robot doesn't get stuck forever.
        // (For this specific problem, a timeout isn't explicitly requested in the loop
        // condition, but it's good practice for more complex scenarios.
        // We will just rely on opMode.opModeIsActive() for now as per instructions).
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        // Step 4: Create the main control loop.
        while (opMode.opModeIsActive()) {
            // Step 4a: Update the odometry readings to get the robot's latest position.
            odo.update(); // Make sure your Odometry class has an update method

            // Step 4b: Get the robot's current X, Y, and Heading.
            double currentX_CM = odo.getPosX(); // Assuming getPosX returns CM
            double currentY_CM = odo.getPosY(); // Assuming getPosY returns CM
            double currentHeadingRad = odo.getHeadingRad(); // Assuming getHeadingRad returns radians

            // Step 4c: Calculate the "error" (how far off you are).
            double dx = xTargetCM - currentX_CM;
            double dy = yTargetCM - currentY_CM;
            double distance = Math.sqrt(dx * dx + dy * dy);
            double headingError = angleWrap(headingTargetRad - currentHeadingRad);

            // Step 4d: Check if the robot is "close enough" to the target.
            if (distance < POSITION_TOLERANCE_CM && Math.abs(headingError) < ANGLE_TOLERANCE_RAD) {
                opMode.telemetry.addData("Status", "Target Reached!");
                opMode.telemetry.update();
                break; // Exit the loop
            }

            // Step 4e: Determine the motor powers using "Bang-Bang" control!
            // Note: The problem specifies 0.2, but using maxPower from the method signature
            // would make this more flexible. For now, sticking to 0.2 as per instructions.
            // Let's use a base power and scale it with maxPower if needed,
            // or just use a fixed small power for bang-bang.
            // The instructions specify 0.2 directly.
            double bangBangPower = 0.2; // Fixed power for bang-bang

            double xPower = (dx > 0) ? bangBangPower : -bangBangPower;
            double yPower = (dy > 0) ? bangBangPower : -bangBangPower;
            double headingPower = (headingError > 0) ? bangBangPower : -bangBangPower;

            // Optional: Cap the individual powers by maxPower if you want to scale them
            // For true bang-bang as described (0.2 or -0.2), this isn't strictly necessary,
            // but if maxPower was meant to be an overall speed limit, you might apply it.
            // The instructions are a bit ambiguous here. If maxPower is an overall limit,
            // you might want to normalize the xPower, yPower, headingPower vector and then
            // scale by maxPower. However, the "0.2 or -0.2" implies fixed magnitude.
            // Let's assume the 0.2 is fixed for now.

            // If the error is very small, we might want to reduce power to avoid overshoot,
            // or just rely on the tolerance. For simple bang-bang, we apply full allowed power.
            // To prevent oscillation when very close but not within tolerance for one axis:
            if (Math.abs(dx) < POSITION_TOLERANCE_CM / 2) xPower = 0; // Reduce jitter on x
            if (Math.abs(dy) < POSITION_TOLERANCE_CM / 2) yPower = 0; // Reduce jitter on y
            if (Math.abs(headingError) < ANGLE_TOLERANCE_RAD / 2) headingPower = 0; // Reduce jitter on heading

            // Step 4f: Send these calculated powers to the robot's drive system.
            // The problem statement says: "passing yPower as axial, xPower as lateral"
            // This implies a robot-centric control scheme where yPower is forward/backward
            // and xPower is left/right strafe.
            drive(yPower, xPower, headingPower);

            // Step 4g: Display helpful information on the Driver Station (telemetry).
            opMode.telemetry.addData("Target", "X: %.2f cm, Y: %.2f cm, H: %.1f deg", xTargetCM, yTargetCM, headingTargetDeg);
            opMode.telemetry.addData("Current", "X: %.2f cm, Y: %.2f cm, H: %.1f deg", currentX_CM, currentY_CM, Math.toDegrees(currentHeadingRad));
            opMode.telemetry.addData("Error", "Dist: %.2f cm, H-Err: %.1f deg", distance, Math.toDegrees(headingError));
            opMode.telemetry.addData("Loop Time (ms)", "%.1f", loopTimer.milliseconds());
            opMode.telemetry.addData("Powers", "Ax(Y): %.2f, Lat(X): %.2f, Yaw: %.2f", yPower, xPower, headingPower);
            opMode.telemetry.update();

            loopTimer.reset(); // Reset for next iteration's time measurement if desired
        }

        // Step 5: Once the loop finishes, stop the robot.
        drive(0, 0, 0);
        opMode.telemetry.addData("Status", "Movement Stopped.");
        opMode.telemetry.update();
    }

    // --- Other methods for your Chassis class ---
    public void setDriveMode(DriveMode mode) {
        this.currentDriveMode = mode;
    }

    public DriveMode getDriveMode() {
        return this.currentDriveMode;
    }
}
