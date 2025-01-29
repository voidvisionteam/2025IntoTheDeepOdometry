package org.firstinspires.ftc.teamcode.voidvision.stagetwo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "Sample",group = "Autonomous")
public class Sample_Base extends Auto_Framework{
    @Override
    public void runOpMode() throws InterruptedException{
        //init Framework
        setupAutoFramework();
        //Define initial Position
        beginPose = new Pose2d(0, 28, 0);

        Action runSpecialTestRun = drive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(21,28-5))
                //.strafeTo(new Vector2d(25+.1,28-5))

                .strafeTo(new Vector2d(2,28))
                .strafeTo(new Vector2d (8,38))//prep!!
                .waitSeconds(2)
                .turn((135/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3,38+2))//drop


                .waitSeconds(2)
                .turn((45/360d)*fullTurn)
                .strafeTo(new Vector2d (8,38))
                //GRAB

                .waitSeconds(2)
                .strafeTo(new Vector2d (8,34))
                .waitSeconds(2)

                .strafeTo(new Vector2d (8,38))//prep!!
                .turn((45/360d)*fullTurn*-1)
                .strafeTo(new Vector2d (8-3,38+2))//drop

                .strafeTo(new Vector2d (8,42))
                .turn((45/360d)*fullTurn)
                .waitSeconds(2)
                .turn((45/360d)*fullTurn*-1)
                .strafeTo(new Vector2d (8-3,38+2))//drop
                .waitSeconds(2)
                .turn((54/360d)*fullTurn)
                .strafeTo(new Vector2d (8,42))
                .turn((54/360d)*fullTurn*-1)
                .strafeTo(new Vector2d (8-3,38+2))//drop
                .strafeTo(new Vector2d (40,33))//park






                //.strafeTo(new Vector2d(33,57+4))

                //.strafeTo(new Vector2d(8,70))
               // .turn((14/360d)*fullTurn*-1)
                //.waitSeconds(.5)

                //.turn((31/360d)*fullTurn-1)
                //.strafeTo(new Vector2d (8-3,38+2))

               // .waitSeconds(2)
               // .strafeTo(new Vector2d (8,38))
               // .turn((45/360d)*fullTurn)
                .waitSeconds(4)



                //.strafeTo(new Vector2d(8,70))
                .turn((45/360d)*fullTurn*-1)
                .waitSeconds(.5)
                .turn((45/360d)*fullTurn)

                .waitSeconds(.2)
                //.strafeTo(new Vector2d(33,57+4+2*(9.25)))

                //.strafeTo(new Vector2d(8,70))
                .turn((135/360d)*fullTurn)
                .waitSeconds(.5)
                .turn((135/360d)*fullTurn*-1)

                .waitSeconds(.2)
                //.strafeTo(new Vector2d(13,63))

                .build();
        Action runSample1 = drive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(2,28))
                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((135/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3,38+2))
                .build();
        Action scoreSample1 = drive.actionBuilder(new Pose2d(5,40,(135/360d)*fullTurn))
                .waitSeconds(.5)
                .build();
        Action Sample1 = new SequentialAction(
                new ParallelAction(
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                new ParallelAction(
                        new SequentialAction(
                                new SequentialAction(
                                        lift14.liftUp(),
                                        //.rotateClawSpec(),
                                        clawServoRotate13.rotateClawHighBasket()
                                )
                        ),
                        runSample1
                ),
                new SequentialAction(clawServo12.openClaw(),scoreSample1)
        );
        Action postSample1a2 = drive.actionBuilder(new Pose2d(5,40,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (5+2,40-2))
                .build();
        Action postSample1a1 = drive.actionBuilder(new Pose2d(5+2,40-2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((45/360d)*fullTurn)
                .strafeTo(new Vector2d (8+7+3,34))
                .build();
        Action scoreSample2 = drive.actionBuilder(new Pose2d(8+7+3,34,(180/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((-45/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3,38+2))
                .build();
        Action scoreSample2wait = drive.actionBuilder(new Pose2d(5,40,(135/360d)*fullTurn))
                .waitSeconds(.5)
                .build();
        Action Sample2 = new SequentialAction(
                postSample1a2,
                new ParallelAction(postSample1a1,lift14.liftDown(),clawServoRotate13.rotateClawHome()),
                clawServoRotate13.rotateClawSpec(),
                Grab(new Pose2d(8+7+3,34,(180/360d)*fullTurn)),
                Transfer(new Pose2d(8+7+3,34,(180/360d)*fullTurn)),
                new ParallelAction(lift14.liftUp(),scoreSample2),
                new SequentialAction(clawServo12.openClaw(),scoreSample2wait)
        );

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(Sample1,Sample2)

        );
    }
}
