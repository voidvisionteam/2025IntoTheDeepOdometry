package org.firstinspires.ftc.teamcode.voidvision.stagetwo.stagethree;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.voidvision.stagetwo.Auto_Framework;

@Autonomous(name = "Sample_Base2",group = "Autonomous")
public class Sample_Base2 extends Auto_Framework2 {
    public MecanumDrive getMecanum(Pose2d start){
        return new MecanumDrive(hardwareMap,start);
    }
    @Override
    public void runOpMode() throws InterruptedException{
        //init Framework
        setupAutoFramework(RevBlinkinLedDriver.BlinkinPattern.GREEN);
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



        Action runSample1 = drive.actionBuilde4(beginPose)
                .strafeTo(new Vector2d(4,28))
                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((135/360d)*fullTurn)
                //.strafeToLinearHeading(new Vector2d (88,38),(135/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+1,38+2+2))
                .build();
        Action scoreSample1 = drive.actionBuilder(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .waitSeconds(0)
                .build();
        Action Sample1 = new SequentialAction(
                new ParallelAction(
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        lift14.liftUp(),
                                        //.rotateClawSpec(),
                                        clawServoRotate13.rotateClawHighBasket()
                                )
                        ),
                        runSample1
                ),
                new SequentialAction(clawServo12.openClaw(),scoreSample1)
        );
        Action postSample1a2 = drive.actionBuilder(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (5+2,40-2))
                .build();
        Action postSample1a1 = drive.actionBuilde3(new Pose2d(5+2,40-2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((45/360d)*fullTurn)
                .strafeTo(new Vector2d (8+7+3+2-2,34))
                .build();
        Action scoreSample2 = drive.actionBuilder(new Pose2d(8+7+3+2,34,(180/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((-45/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+2,38+2+2))
                .build();
        Action scoreSample2wait = drive.actionBuilder(new Pose2d(5+1,40+2+0.5,(135/360d)*fullTurn))
                .waitSeconds(.0)
                .build();
        Action Sample2 = new SequentialAction(
                postSample1a2,
                new ParallelAction(postSample1a1,lift14.liftDown(),clawServoRotate13.rotateClawHome()),
                clawServoRotate13.rotateClawSpec(),
                Grab(new Pose2d(8+7+3+2-2,34,(180/360d)*fullTurn)),
                Transfer(new Pose2d(8+7+3+2-2,34,(180/360d)*fullTurn),true),
                //new ParallelAction(lift14.liftUp(),scoreSample2),
                new SequentialAction(clawServo12.openClaw(),scoreSample2wait)
        );

        Action postSample2a2 = drive.actionBuilder(new Pose2d(5+1,40+2+.5,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (5+2,40-2))
                .build();
        Action postSample2a1 = drive.actionBuilde3(new Pose2d(5+2,40-2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((45/360d)*fullTurn)
                .strafeTo(new Vector2d (8+7+3+2-2,34+9.4*(1) -1+1))
                .build();
        Action scoreSample3 = drive.actionBuilder(new Pose2d(8+7+3+2,34+9.4*(1)+1,(180/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((-45/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+1,38+2+2))
                .build();
        Action scoreSample3wait = drive.actionBuilder(new Pose2d(5+1,40+2+0.5,(135/360d)*fullTurn))
                .waitSeconds(0)
                .build();
        Action Sample3 = new SequentialAction(
                postSample2a2,
                new ParallelAction(postSample2a1,lift14.liftDown(),clawServoRotate13.rotateClawHome()),
                clawServoRotate13.rotateClawSpec(),
                Grab(new Pose2d(8+7+3+2-2,34+9.4*(1)-1+1,(180/360d)*fullTurn)),
                Transfer(new Pose2d(8+7+3+2-2,34+9.4*(1)-1+1,(180/360d)*fullTurn),true),
                //new ParallelAction(lift14.liftUp(),scoreSample3),
                new SequentialAction(clawServo12.openClaw(),scoreSample3wait)
        );

        Action postSample3a2 = drive.actionBuilder(new Pose2d(5+1,40+2+.5,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (5+2,40-2))
                .build();
        Action postSample3a1 = drive.actionBuilde3(new Pose2d(5+2,40-2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn(((45+30+5)/360d)*fullTurn)
                .strafeTo(new Vector2d (8+7+3+2+1-1+1,34+9.4*(1)+1+.25 -1))
                .build();
        Action scoreSample4 = drive.actionBuilder(new Pose2d(8+7+3+2+1,34+9.4*(1)+1+.25,(215/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((-(45+30+5)/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+1,38+2+2))
                .build();
        Action scoreSample4wait = drive.actionBuilde3(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .waitSeconds(.6)
                .strafeTo(new Vector2d (8,38))
                .build();
        Action Sample4 = new SequentialAction(
                postSample3a2,
                new ParallelAction(postSample3a1,lift14.liftDown(),clawServoRotate13.rotateClawHome()),
                clawServoRotate13.rotateClawSpec(),
                Grab(new Pose2d(8+7+3+2+1-1+1,34+9.4*(1)+1+.25-1,(215/360d)*fullTurn)),
                TransferAlt(new Pose2d(8+7+3+2+1-1+1,34+9.4*(1)+1+.25-1,(215/360d)*fullTurn),true),
                //new ParallelAction(lift14.liftUp(),scoreSample4),
                new SequentialAction(clawServo12.openClaw(),scoreSample4wait)
        );



        Action StateRun = drive.actionBuilder(beginPose)

                .strafeTo(new Vector2d(4,28))


                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((135/360d)*fullTurn)
                //.strafeToLinearHeading(new Vector2d (88,38),(135/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+1,38+2+2))

                .turn((45/360d)*fullTurn)

                .strafeTo(new Vector2d (18,23.4))
                .strafeTo(new Vector2d (51,23.4))
                .strafeTo(new Vector2d (51,34.4))
                .strafeTo(new Vector2d (21,34.4))

                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((135/360d)*fullTurn)
                //.strafeToLinearHeading(new Vector2d (88,38),(135/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+1,38+2+2))

                .turn((45/360d)*fullTurn)

                .strafeTo(new Vector2d (18,23.4+9.5*(1)))
                .strafeTo(new Vector2d (51,23.4+9.5*(1)))
                .strafeTo(new Vector2d (51,34.4+9.5*(1)))
                .strafeTo(new Vector2d (21,34.4+9.5*(1)))

                //.strafeTo(new Vector2d (8+7+3+2-2,34+9.4*(1) -1+1))

                .build();


        Action NewSample1run = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(4,28))
                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((135/360d)*fullTurn)
                .strafeTo(new Vector2d (6,41.5))
                .build();

        Action NewCollectSample2 = drive.actionBuilde3(new Pose2d(7,38,(180/360d)*fullTurn))
                .strafeTo(new Vector2d (18,23.4))
                .strafeTo(new Vector2d (51,23.4))
                .strafeTo(new Vector2d (51,34.4))
                //.strafeTo(new Vector2d (21,34.4))
                .build();
        Action NewScoreSample2 = drive.actionBuilde3(new Pose2d(21,34.4,(180/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((-45/360d)*fullTurn)
                .strafeTo(new Vector2d (6,42))
                .build();

        Action NewCollectSample3 = drive.actionBuilde3(new Pose2d(7,38,(180/360d)*fullTurn))
                .strafeTo(new Vector2d (18,23.4+9.5*(1)))
                .strafeTo(new Vector2d (51,23.4+9.5*(1)))
                .strafeTo(new Vector2d (51,34.4+9.5*(1)))
                //.strafeTo(new Vector2d (21,34.4+9.5*(1)))
                .build();
        Action NewScoreSample3 = drive.actionBuilder(new Pose2d(21,34.4+9.5*(1),(180/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((-45/360d)*fullTurn)
                .strafeTo(new Vector2d (6,42))
                .build();

        Action NewCollectSample4 = drive.actionBuilder(new Pose2d(7,38,(180/360d)*fullTurn))
                .turn(((30)/360d)*fullTurn)
                .strafeTo(new Vector2d (8+7+3+2+1-1+1,34+9.4*(1)+1+.25 -1))
                .build();
        Action NewScoreSample4 = drive.actionBuilder(new Pose2d(8+7+3+2+1-1+1,34+9.4*(1)+1+.25-1,(215/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))//prep!!
                //.waitSeconds(2)
                .turn((-45/360d)*fullTurn)
                .strafeTo(new Vector2d (6,42))
                .build();
        Action NewSample1 = new SequentialAction(
                new ParallelAction(
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                new ParallelAction(
                        lift14.liftUp(),
                        //.rotateClawSpec(),
                        clawServoRotate13.rotateClawHighBasket(),
                        NewSample1run
                ),
                new SequentialAction(
                        clawServo12.openClaw(),
                        ScoreSampleWaitAction(true,true,true),
                        PostSampleBackupAction(true,true,true)
                )
        );
        Action NewSample2 = new SequentialAction(
                new ParallelAction(
                        NewCollectSample2,
                        lift14.liftDown(),
                        clawServoRotate13.rotateClawHome()
                ),
                //Grab(new Pose2d(21,34.4,(180/360d)*fullTurn)),
                NewTransfer(new Pose2d(51,34.4,(180/360d)*fullTurn),true),
                new SequentialAction(
                        clawServo12.openClaw(),
                        ScoreSampleWaitAction(true,true,true),
                        PostSampleBackupAction(true,true,true)
                )
        );
        Action NewSample3 = new SequentialAction(
                new ParallelAction(
                        NewCollectSample3,
                        lift14.liftDown(),
                        clawServoRotate13.rotateClawHome()
                ),
                //Grab(new Pose2d(21,34.4,(180/360d)*fullTurn)),
                NewTransfer(new Pose2d(51,34.4+9.5*(1),(180/360d)*fullTurn),true),
                new SequentialAction(
                        clawServo12.openClaw(),
                        ScoreSampleWaitAction(true,true,true),
                        PostSampleBackupAction(true,true,true)
                )
        );
        Action NewSample4 = new SequentialAction(
                new ParallelAction(
                        NewCollectSample4,
                        lift14.liftDown(),
                        clawServoRotate13.rotateClawHome()
                ),
                Grab(new Pose2d(8+7+3+2+1-1+1,34+9.4*(1)+1+.25-1,(215/360d)*fullTurn)),
                TransferAlt(new Pose2d(8+7+3+2+1-1+1,34+9.4*(1)+1+.25-1,(215/360d)*fullTurn),true),
                ScoreSampleWaitAction(true,true,true),
                PostSampleBackupAction(true,true,true)
        );


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(NewSample1,NewSample2,NewSample3,NewSample4)

        );
    }

}

