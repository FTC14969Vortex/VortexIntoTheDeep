package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

import com.qualcomm.robotcore.util.ElapsedTime;


public class Autonomous {
    // Method signature provided for you. Don't change this line.
    public void goToPosition(double xTargetCM, double yTargetCM, double headingTargetDeg, double maxPower) {
        // Step 1: Convert the target heading from degrees to radians.// Step 2: Define your "tolerances" – how close is close enough to stop.
// Use the values from the example:
//    POSITION_TOLERANCE_CM should be 2.0
//    ANGLE_TOLERANCE_RAD should convert 3 degrees to radians


        // Step 2: Define your "tolerances" – how close is close enough to stop.
        // Use the values from the example:
        //    POSITION_TOLERANCE_CM should be 2.0
        //    ANGLE_TOLERANCE_RAD should convert 3 degrees to radians
        double POSITION_TOLERANCE_CM = 2.0;
        double ANGLE_TOLERANCE_RAD = AngleUnit.RADIANS.fromDegrees(3.0);
        double headingTargetRad = AngleUnit.RADIANS.fromDegrees(headingTargetDeg);
        // Step 3: Set up a timer to make sure your robot doesn't get stuck forever.
        // Initialize an ElapsedTime object and reset it.
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        // Step 4: Create the main control loop. The robot will keep doing these steps
        // until the OpMode is stopped or it reaches the target.
        // The loop should continue as long as the OpMode is active.
        // Step 4a: Update the odometry readings to get the robot's latest position.
        while (opModeIsActive()) {
            GoBildaPinpointDriver odo;
            Pose2D position = odo.getPosition();
            odo.update();
            // Step 4b: Get the robot's current X, Y, and Heading.
            // Remember that `odo.getPosX` and `odo.getPosY` can get values in CM.
            // Get the current heading in Radians.

            // Step 4c: Calculate the "error" (how far off you are) for X, Y, and Heading.
            //   - Calculate `dx` (difference in X between target and current).
            //   - Calculate `dy` (difference in Y between target and current).
            //   - Calculate `distance` (straight-line distance to target using dx and dy).
            //   - Calculate `headingError` (difference in heading, remember to use `angleWrap`!).

            // Step 4d: Check if the robot is "close enough" to the target.
            // If the `distance` is less than `POSITION_TOLERANCE_CM` AND the absolute `headingError`
            // is less than `ANGLE_TOLERANCE_RAD`, then exit the loop.

            // Step 4e: Determine the motor powers using "Bang-Bang" control!
            // You'll set a fixed power (like 0.2 or -0.2) based on the sign of the error.
            //   - For `xPower`: If `dx` is positive, set `xPower` to 0.2 (move right); otherwise, set to -0.2 (move left).
            //   - For `yPower`: If `dy` is positive, set `yPower` to 0.2 (move forward); otherwise, set to -0.2 (move backward).
            //   - For `headingPower`: If `headingError` is positive, set `headingPower` to 0.2 (turn counter-clockwise);
            //     otherwise, set to -0.2 (turn clockwise).

            // Step 4f: Send these calculated powers to the robot's drive system.
            // Call the `drive` method, passing `yPower` as axial, `xPower` as lateral, and `headingPower` as yaw.

            // Step 4g: Display helpful information on the Driver Station (telemetry).
            // Use `opMode.telemetry.addData` to show:
            //   - Target X and Y.
            //   - Current X and Y.
            //   - Current distance error.
            //   - Current heading error (converted back to degrees for readability).
            // Call `opMode.telemetry.update()` to send the data.

            // Step 5: Once the loop finishes (either target reached or OpMode stopped),
            // stop the robot completely by calling the `drive` method with zero power for all directions.
        }
        double inch2mm = 25.4;
        double driveSpeed = 0.2;
        int timeoutMs = 3000;

        Pose2D startPose = chassis.getPoseEstimate();
        double x0 = startPose.getX(DistanceUnit.MM);
        double y0 = startPose.getY(DistanceUnit.MM);
        double heading0 = startPose.getHeading(AngleUnit.RADIANS);
    }

    private boolean opModeIsActive() {
        return false;
    }
}
