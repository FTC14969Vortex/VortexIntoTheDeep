package org.firstinspires.ftc.teamcode;
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
    float DRIVE_SPEED = 0.8F;
    float TURN_OFFSET = 10;
    //squareDistance is the length of one square in the 12 foot by 12 foot playing area
    float squareDistance = 24;


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
                robot.chassis.Drive(DRIVE_SPEED, 1.5f*squareDistance);
                sleep(1000);
                robot.chassis.autoTurn(-95,TURN_OFFSET);
                sleep(1000);
                robot.chassis.Drive(DRIVE_SPEED, squareDistance);
                sleep(1000);
                currentStage = AutoStages.HANG_SPECIMEN;
                break;
            case HANG_SPECIMEN:
                robot.arm.gotoHighPosition();
                sleep(1000);
                int highPosition = -1450;
//                robot.arm.gotoPosition(highPosition-50);
//                sleep(1000);
                double intakeSpeed = 1.0;
                robot.intake.MoveIntake(intakeSpeed);
                currentStage = AutoStages.LOWER_ARM;
                break;
            case LOWER_ARM:
                robot.chassis.Drive(DRIVE_SPEED, -squareDistance);
                robot.arm.gotoPickupPosition();
                currentStage = AutoStages.GET_SAMPLE;
                break;
            case GET_SAMPLE:
               robot.chassis.Strafe(DRIVE_SPEED,2.5F*squareDistance);
                robot.chassis.Drive(DRIVE_SPEED,0.5F*squareDistance);
                currentStage = AutoStages.DROP_AT_BASKET;
                break;
            case DROP_AT_BASKET:
                robot.chassis.Drive(DRIVE_SPEED,squareDistance);
                robot.chassis.autoTurn(-202.5F, TURN_OFFSET);
                robot.arm.gotoHighPosition();
                robot.chassis.autoTurn(-135F,TURN_OFFSET);
                robot.arm.gotoPickupPosition();
                robot.chassis.Strafe(DRIVE_SPEED,0.5F*squareDistance);
                robot.chassis.Drive(DRIVE_SPEED,1.25F*squareDistance);
                currentStage = AutoStages.PREPARE_FOR_TELEOP;
                break;
            case PREPARE_FOR_TELEOP:
                robot.chassis.stopDriveMotors();

                break;
        }
    }
}