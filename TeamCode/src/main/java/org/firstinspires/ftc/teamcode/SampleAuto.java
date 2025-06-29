package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.Chassis;

@Autonomous(name = "SampleAuto_Mecanum", group = "Auto")
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis();
        chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
        chassis.init(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Move chassis
        double maxPower = 0.5;
        chassis.goToPosition(100, 50, 90, maxPower);
    }
}