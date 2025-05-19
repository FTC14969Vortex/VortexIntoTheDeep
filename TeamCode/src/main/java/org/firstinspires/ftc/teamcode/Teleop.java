//imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {
//
//    public DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
//
//    @Override
//    public void runOpMode() throws InterruptedException{
//
//        motor.setDirection(DcMotor.Direction.FORWARD);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            double power = gamepad1.left_stick_y;
//            // Set the power of the motor
//            motor.setPower(power);
//
//        }
//
//    }

public DcMotor  motor   = null;
@Override
    public void runOpMode() {
    double power;

    motor = hardwareMap.get(DcMotor.class, "motor");

    motor.setDirection(DcMotor.Direction.FORWARD);

    waitForStart();

//    while (opModeIsActive()) {
        motor.setPower(1);
        sleep(5000);
        motor.setPower(0);

        telemetry.addData("Encoder Position", motor.getCurrentPosition());
        telemetry.update();

        sleep(5000);
}
}