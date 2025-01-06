package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.Robot;



@Autonomous(name = "Auto Right Basket", group = "Auto")
public class AutoRightBasket extends LinearOpMode{
    //------------------------------------------------------------------

    //Robot Object
    public Robot robot = new Robot(this);

    // Robot control parameters
    float DRIVE_SPEED = 0.8F;
    float TURN_OFFSET = 10;
    //TILE_LENGTH is the length of one square in the 12 foot by 12 foot playing area
    final float TILE_LENGTH = 24;
    double drivePower = 0.8;
    double holdTime = 0.125;


    enum AutoStages {
        MOVE_TO_BASKET,
        RAISE_ARM,
        DROP_SAMPLE,
        LOWER_ARM,
        PARK,
    }

    AutoStages currentStage = AutoStages.MOVE_TO_BASKET;

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
        robot.init();

        //Reset all chassis settings to ensure they are correct
        robot.chassis.resetAll();

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

        //TODO: make switch case with new drive methods

        // Code to run after the driver hits PLAY
        switch (currentStage) {
            case MOVE_TO_BASKET:
                robot.chassis.drive(-0.65*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(90,drivePower,holdTime);
                robot.chassis.drive(-2.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(125,drivePower,holdTime);

                currentStage = AutoRightBasket.AutoStages.RAISE_ARM;

            case RAISE_ARM:
                robot.arm.gotoLowBox();
                robot.chassis.drive(-0.55*TILE_LENGTH,drivePower,holdTime);

                currentStage = AutoRightBasket.AutoStages.DROP_SAMPLE;

            case DROP_SAMPLE:
                robot.intake.MoveIntake(-1);
                sleep(1000);
                robot.intake.stopIntake();

                currentStage = AutoRightBasket.AutoStages.LOWER_ARM;

            case LOWER_ARM:
                robot.arm.gotoPosition(0);
                robot.chassis.drive(0.55*TILE_LENGTH,drivePower,holdTime);

                currentStage = AutoRightBasket.AutoStages.PARK;

            case PARK:
                robot.chassis.turnTo(0, drivePower, holdTime);
                robot.chassis.drive(-4*TILE_LENGTH,drivePower,holdTime);

        }
    }
}