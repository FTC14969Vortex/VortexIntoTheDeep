//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.SampleMechanism;
@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {
    public DcMotor motor = null;
    public SampleMechanism mechanism = new SampleMechanism(hardwareMap);

    @Override
    public void runOpMode() {
        double power;
        double mechanismPower;

        motor = hardwareMap.get(DcMotor.class, "backLeftDrive");

        motor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

            while (opModeIsActive()) {

                power = gamepad1.right_stick_x;
                mechanismPower = gamepad1.left_stick_x;
                motor.setPower(power);
                mechanism.motor.setPower(mechanismPower);

                telemetry.addData("Encoder Position", motor.getCurrentPosition());
                telemetry.addData("Mechanism Position", mechanism.motor.getCurrentPosition());

                telemetry.update();
            }
    }
}