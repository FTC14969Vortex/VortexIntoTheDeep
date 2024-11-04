package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helper.Robot;

@Autonomous(name = "Auto Common", group = "Auto")
public class AutoCommon extends LinearOpMode{
    //------------------------------------------------------------------

    //Robot Object
    public Robot robot = new Robot();

    // Robot control parameters
    double DRIVE_SPEED = 0.8;
    float TURN_OFFSET = 10;

    enum AutoStages {
        MOVE_TO_SUBMERSIBLE,
        HANG_SPECIMEN,
        LOWER_ARM,
        GET_SAMPLE,
        DROP_AT_BASKET,
        PREPARE_FOR_TELEOP
    }

    AutoStages currentStage = AutoStages.MOVE_TO_SUBMERSIBLE;

    public double SUB_POS_X = 0;
    public double SUB_POS_Y = 0;
    public double BASKET_POS_X = 0;
    public double BASKET_POS_Y = 0;
    public double PARK_POS_X = 0;
    public double PARK_POS_Y = 0;



    /**
     * Set parameters specific to starting position in Auto here.
     */
    public void setUniqueParameters() {
        SUB_POS_X = 0;
        SUB_POS_Y = 0;
        BASKET_POS_X = 0;
        BASKET_POS_Y = 0;
        PARK_POS_X = 0;
        PARK_POS_Y = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code here
        robot.init(hardwareMap);




        //Wait for the play button to be pressed
        waitForStart();

        /* Functions to use to control robot:
         * Chassis:
         * robot.chassis.Drive(Speed, Distance);
         * robot.chassis.Strafe(Speed, Distance);
         * robot.chassis.autoTurn(Angle, Offset);
         * robot.chassis.stopDriveMotors();
         *
         * Arm:
         * robot.arm.goToPosition(Position);
         *
         * Intake:
         * robot.intake.MoveIntake(speed);
         * robot.intake.stopIntake();
         *
         */

        // Use above functions to control robot and use flow chart in Slack to determine how to move and what to do.
        //It's okay if the code isn't perfect, we can review and fix it later as a group

        //If a mechanism does not have code written for it yet, please write a comment in the code to indicate that it needs to be written.

        //The intake and wrist code work for last years robot, we will edit it later for this years robot once it is built.


        // Code to run after the driver hits PLAY
        switch (currentStage) {
            case MOVE_TO_SUBMERSIBLE:
                //Lam will add code here
                float x_distance = 91.44F;
                robot.chassis.Drive(DRIVE_SPEED, x_distance);
                sleep(1000);
                robot.chassis.autoTurn(90,TURN_OFFSET);
                float y_distance = 91.44F;
                robot.chassis.Drive(DRIVE_SPEED, y_distance);
                sleep(1000);
                robot.chassis.stopDriveMotors();


                currentStage = AutoStages.HANG_SPECIMEN;
                break;
            case HANG_SPECIMEN:
                //Lam will add code here
                robot.arm.goToPosition(Position);
                robot.wrist.goToPosition(Position);
                sleep(1000);
                robot.arm.goToPosition(Position);
                sleep(1000);
                robot.outtake.MoveOuttake(speed);
                robot.outtake.stopOuttake();

                currentStage = AutoStages.LOWER_ARM;
                break;
            case LOWER_ARM:
                //Tanishk will add code here
                currentStage = AutoStages.GET_SAMPLE;
                break;
            case GET_SAMPLE:
                //Tanishk will add code here
                currentStage = AutoStages.DROP_AT_BASKET;
                break;
            case DROP_AT_BASKET:
                //Chris will add code here
                currentStage = AutoStages.PREPARE_FOR_TELEOP;
                break;
            case PREPARE_FOR_TELEOP:
                //Chris will add code here
                break;
        }
    }
}
