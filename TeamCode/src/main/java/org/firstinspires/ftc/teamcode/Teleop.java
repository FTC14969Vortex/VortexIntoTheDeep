//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    public DcMotor motor = null;
    @Override

    public void runOpMode() {
        double power;

        motor = hardwareMap.get(DcMotor.class, "backLeftDrive");

        motor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

            while (opModeIsActive()) {

                power = gamepad1.right_stick_x;
                motor.setPower(power);

                telemetry.addData("Encoder Position", motor.getCurrentPosition());
                telemetry.update();
            }
    }
}