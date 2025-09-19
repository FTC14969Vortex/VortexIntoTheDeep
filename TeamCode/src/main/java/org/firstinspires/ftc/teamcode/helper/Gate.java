package org.firstinspires.ftc.teamcode.helper;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Teleop;

public class Gate {

    private CRServo leftGate;
    private CRServo rightGate;
    private OpMode opMode;

    public void init(OpMode opMode) {

        this.opMode = opMode;

        leftGate = opMode.hardwareMap.get(CRServo.class, "leftGate");
        rightGate = opMode.hardwareMap.get(CRServo.class, "rightGate");
        rightGate.setDirection(CRServo.Direction.REVERSE);;
    }
    public void release(double power) {

        leftGate.setPower(power);
        rightGate.setPower(power);
    }
    public void stop(){

        leftGate.setPower(0);
        rightGate.setPower(0);
    }
    }

