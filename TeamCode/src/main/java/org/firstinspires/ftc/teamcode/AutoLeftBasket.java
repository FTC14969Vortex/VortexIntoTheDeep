package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.Robot;



@Autonomous(name = "Auto Left Basket", group = "Auto")
public class AutoLeftBasket extends LinearOpMode{
    //------------------------------------------------------------------

    //Robot Object
    public Robot robot = new Robot(this);

    // Robot control parameters
    float DRIVE_SPEED = 1F;
    float TURN_OFFSET = 10;
    //TILE_LENGTH is the length of one square in the 12 foot by 12 foot playing area
    final float TILE_LENGTH = 24;
    double drivePower = 0.8;
    double holdTime = 0.025;


    enum AutoStages {
        DROP_SAMPLE1,
        PICK_UP_SAMPLE2,
        DROP_SAMPLE2,
        PICK_UP_SAMPLE3,
        DROP_SAMPLE3,
        PICK_UP_SAMPLE4,
        DROP_SAMPLE4,
        PARK,

    }

    AutoStages currentStage = AutoStages.DROP_SAMPLE1;

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
            case DROP_SAMPLE1:

                robot.chassis.drive(-1.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(225,drivePower,holdTime);
                robot.chassis.drive(0.75*TILE_LENGTH,drivePower,holdTime);
                robot.arm.gotoLowBox();
                robot.wrist.turnToStraightPosition();
                robot.slider.gotoSliderHighBoxPosition();
                robot.intake.MoveIntake(-1);
                sleep(3000);
                robot.intake.stopIntake();

                currentStage = AutoLeftBasket.AutoStages.PICK_UP_SAMPLE2;

            case PICK_UP_SAMPLE2:

                robot.slider.gotoSliderPickUpPosition();
                robot.arm.gotoPickUpPosition();
                robot.chassis.drive(-0.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(180,drivePower,holdTime);
                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.intake.MoveIntake(-1);
                sleep(3000);
                robot.intake.stopIntake();

                currentStage = AutoLeftBasket.AutoStages.DROP_SAMPLE2;

            case DROP_SAMPLE2:

                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(90,drivePower,holdTime);
                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.arm.gotoHighBox();
                robot.slider.gotoSliderHighBoxPosition();
                robot.intake.MoveIntake(-1);
                sleep(3000);
                robot.intake.stopIntake();

                currentStage = AutoLeftBasket.AutoStages.PICK_UP_SAMPLE3;

            case PICK_UP_SAMPLE3:

                robot.slider.gotoSliderPickUpPosition();
                robot.arm.gotoPickUpPosition();
                robot.chassis.drive(-0.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(180,drivePower,holdTime);
                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.intake.MoveIntake(-1);
                sleep(3000);
                robot.intake.stopIntake();

                currentStage = AutoLeftBasket.AutoStages.DROP_SAMPLE3;

            case DROP_SAMPLE3:

                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(90,drivePower,holdTime);
                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.arm.gotoHighBox();
                robot.slider.gotoSliderHighBoxPosition();
                robot.intake.MoveIntake(-1);
                sleep(3000);
                robot.intake.stopIntake();

                currentStage = AutoLeftBasket.AutoStages.PICK_UP_SAMPLE4;

            case PICK_UP_SAMPLE4:

                robot.slider.gotoSliderPickUpPosition();
                robot.arm.gotoPickUpPosition();
                robot.chassis.drive(-0.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(180,drivePower,holdTime);
                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.intake.MoveIntake(-1);
                sleep(3000);
                robot.intake.stopIntake();

                currentStage = AutoLeftBasket.AutoStages.DROP_SAMPLE4;
            case DROP_SAMPLE4:

                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.turnTo(90,drivePower,holdTime);
                robot.chassis.drive(0.25*TILE_LENGTH,drivePower,holdTime);
                robot.arm.gotoHighBox();
                robot.slider.gotoSliderHighBoxPosition();
                robot.intake.MoveIntake(-1);
                sleep(3000);
                robot.intake.stopIntake();

                currentStage = AutoLeftBasket.AutoStages.PARK;
            case PARK:
                robot.chassis.turnTo(90, drivePower, holdTime);
                robot.chassis.drive(3.5*TILE_LENGTH,drivePower,holdTime);
                robot.chassis.strafe(TILE_LENGTH, drivePower, holdTime);
        }
    }
}