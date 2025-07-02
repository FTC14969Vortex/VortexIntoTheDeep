package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Chassis {
    enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    void init(OpMode opMode) {

    }

    void setDriveMode(DriveMode mode) {

    }
    // gamepad B

    void resetIMU() {

    }
    // gamepad A

    // Print out
    // - Driving Mode
    // - Odometry computer's reading of (heading, x, y)
    void updateTelemetry() {

    }
    // TeleOp Mode Methods

    /**
     * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
     * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
     * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
     */
    void drive(double axial, double lateral, double yaw) {

    }
}