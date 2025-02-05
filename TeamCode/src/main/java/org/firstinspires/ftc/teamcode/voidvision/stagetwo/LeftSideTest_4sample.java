package org.firstinspires.ftc.teamcode.voidvision.stagetwo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "1 spec 3 sample",group = "Autonomous")
public class LeftSideTest_4sample extends Auto_Framework{
    public MecanumDrive getMecanum(Pose2d start){
        return new MecanumDrive(hardwareMap,start);
    }
    @Override
    public void runOpMode() throws InterruptedException{
        //init Framework
        setupAutoFramework(RevBlinkinLedDriver.BlinkinPattern.BLUE);
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
        Action runToHighBar = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,28-5))
                .strafeTo(new Vector2d(25+.1,28-5))
                .build();
        Action Speciman1 = new SequentialAction(
        new ParallelAction(
                runToHighBar,
                new SequentialAction(
                        new ParallelAction(
                                lift14.liftUpSpecialHeight(),
                                clawServoRotate13.rotateClawHome()
                        ),
                        new ParallelAction(
                                lift14.liftUpB(),
                                clawServoRotate13.rotateClawSpec()
                        )


                )

        ),
        new ParallelAction(
                lift14.liftDown(),
                clawServoRotate13.rotateClawHome()
        )
        );





        Action postSpeciman1a2 = drive.actionBuilder(new Pose2d(25+.1,28-5,(0/360d)*fullTurn))
                .strafeTo(new Vector2d(23,4+57))
                .strafeTo(new Vector2d(32,57+4))
                .build();
        Action postSample1a1 = drive.actionBuilder(new Pose2d(32,57+4,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((-135/360d)*fullTurn)
                .strafeTo(new Vector2d (30,34))
                .build();
        Action scoreSample2 = drive.actionBuilder(new Pose2d(30,34,(0/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))//ADD TUrn while drive here
                .turn((135/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+2,38+2+2))
                .build();
        Action scoreSample2wait = drive.actionBuilder(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .waitSeconds(.0)
                .build();
        Action Sample2 = new SequentialAction(
                postSpeciman1a2,
                new ParallelAction(lift14.liftDown(),clawServoRotate13.rotateClawHome()),
                clawServo12.closeClaw(),
                new ParallelAction(lift14.liftUp(),scoreSample2),
                new SequentialAction(clawServo12.openClaw(),scoreSample2wait)
        );

        Action postSample2a2 = drive.actionBuilder(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (5+2,40-2))
                .build();
        Action postSample2a1 = drive.actionBuilder(new Pose2d(5+2,40-2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((45/360d)*fullTurn)
                .strafeTo(new Vector2d (8+7+3+2,34+9.4*(1) ))
                .build();
        Action scoreSample3 = drive.actionBuilder(new Pose2d(8+7+3+2,34+9.4*(1),(180/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((-45/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+1,38+2+2))
                .build();
        Action scoreSample3wait = drive.actionBuilder(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .waitSeconds(0)
                .build();
        Action Sample3 = new SequentialAction(
                postSample2a2,
                new ParallelAction(postSample2a1,lift14.liftDown(),clawServoRotate13.rotateClawHome()),
                clawServoRotate13.rotateClawSpec(),
                Grab(new Pose2d(8+7+3+2,34+9.4*(1),(180/360d)*fullTurn)),
                Transfer(new Pose2d(8+7+3+2,34+9.4*(1),(180/360d)*fullTurn),true),
                //new ParallelAction(lift14.liftUp(),scoreSample3),
                new SequentialAction(clawServo12.openClaw(),scoreSample3wait)
        );

        Action postSample3a2 = drive.actionBuilder(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (5+2,40-2))
                .build();
        Action postSample3a1 = drive.actionBuilder(new Pose2d(5+2,40-2,(135/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn(((45+30)/360d)*fullTurn)
                .strafeTo(new Vector2d (8+7+3+2+1,34+9.4*(1)+1+.25 ))
                .build();
        Action scoreSample4 = drive.actionBuilder(new Pose2d(8+7+3+2+1,34+9.4*(1)+1+.25,(210/360d)*fullTurn))
                .strafeTo(new Vector2d (8,38))
                .turn((-(45+30)/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3+1,38+2+2))
                .build();
        Action scoreSample4wait = drive.actionBuilder(new Pose2d(5+1,40+2,(135/360d)*fullTurn))
                .waitSeconds(0)
                .build();
        Action Sample4 = new SequentialAction(
                postSample3a2,
                new ParallelAction(postSample3a1,lift14.liftDown(),clawServoRotate13.rotateClawHome()),
                clawServoRotate13.rotateClawSpec(),
                Grab(new Pose2d(8+7+3+2+1,34+9.4*(1)+1+.25,(210/360d)*fullTurn)),
                TransferAlt(new Pose2d(8+7+3+2+1,34+9.4*(1)+1+.25,(210/360d)*fullTurn),true),
                //new ParallelAction(lift14.liftUp(),scoreSample4),
                new SequentialAction(clawServo12.openClaw(),scoreSample4wait)
        );

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(Speciman1,Sample2,Sample3,Sample4)

        );
    }
}
