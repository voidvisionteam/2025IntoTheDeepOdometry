package org.firstinspires.ftc.teamcode.voidvision.stagetwo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.voidvision.redLeftAuto;

public class Red_Left_Sample extends Auto_Framework {
    @Override
    public void runOpMode() throws InterruptedException {
        //init Framework
        setupAutoFramework(RevBlinkinLedDriver.BlinkinPattern.RED);
        //Define initial Position
        beginPose = new Pose2d(0, 28, 0);
        //Reorient Claw?

        //Define Actions
        Action run1 = drive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(21,6))
                //.strafeTo(new Vector2d(21,28))
                .strafeTo(new Vector2d(21,28-5))
                .build();
        Action run2 = drive.actionBuilder(new Pose2d(21,28-5,0))
                .strafeTo(new Vector2d(25+.1,28-5))
                .build();
        Action run3 = drive.actionBuilder(new Pose2d(25+.1,28-5,0))
                //.strafeTo(new Vector2d(0,6))
                .strafeTo(new Vector2d(23,4+57))
                .strafeTo(new Vector2d(32,57+4))
                .waitSeconds(.25)
                .build();
        Action runwait = drive.actionBuilder(new Pose2d(32,61,0)).waitSeconds(.5).build();
        Action run4 = drive.actionBuilder(new Pose2d(32,61,0))
                //.turn((110/360)*fullTurn)
                //.strafeTo(new Vector2d(-10,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .build();
        Action run5 = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        Action run5back = drive.actionBuilder(new Pose2d(8,70,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(13,63))
                .build();
        Action run6 = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .turn((135/360d)*fullTurn)
                .build();
        Action run7 = drive.actionBuilder(new Pose2d(10,68,(-.5*Math.PI)))
                .strafeTo(new Vector2d(0,-6))
                .build();
        Action run8 = drive.actionBuilder(new Pose2d(0,-6,(-.5*Math.PI)))
                .strafeTo(new Vector2d(95,0))
                .build();
        Action runTest = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(10,0))
                .turn(.25*fullTurn)
                .build();
        Action runTest2 = drive.actionBuilder(beginPose)
                .waitSeconds(.5)
                .build();
        Action runwait2 = drive.actionBuilder(beginPose)
                .waitSeconds(2)
                .build();
        Action runwait3 = drive.actionBuilder(beginPose)
                .waitSeconds(.5)
                .build();
        Action run6alt = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .turn((-135/360d)*fullTurn)
                //.strafeTo(new Vector2d(13,57+5))
                .strafeTo(new Vector2d(13,57+5+9+(5/8d)))
                .strafeTo(new Vector2d(32,57+5+9+(5/8d)))
                .build();
        Action run7alt = drive.actionBuilder(new Pose2d(32,57+5+9+(5/8d),0)).waitSeconds(.5).build();
        Action run8alt = drive.actionBuilder(new Pose2d(32,57+5+9+(5/8d),0))
                //.turn((110/360)*fullTurn)
                //.strafeTo(new Vector2d(-10,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .build();
        Action run9alt = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        Action run9altback = drive.actionBuilder(new Pose2d(8,70,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(13,63))
                .build();
        Action run1a2Right = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,0))
                .strafeTo(new Vector2d(25+.1,6))
                .build();
        Action run1a2Left = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,28-5))
                .strafeTo(new Vector2d(25+.1,28-5))
                .build();
        Action run4a5= drive.actionBuilder(new Pose2d(32,61,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        Action run8a9 = drive.actionBuilder(new Pose2d(35,57+5+9+(5/8d),0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();

        //TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();
        Action runToHighBar = new SequentialAction(
                new ParallelAction(
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                new ParallelAction(
                        run1a2Left,
                        clawServoRotate13.rotateClawSpec(),
                        lift14.liftUpB()
                )
        );
        Action scoreOnHighBar = lift14.liftDown();
        ParallelAction runToSampleOne = new ParallelAction(run3,clawServoRotate13.rotateClawHome(),clawServo12.openClaw());
        ParallelAction retrieveSampleOne = new ParallelAction(runwait,clawServo12.closeClaw());
        ParallelAction runToBasketOne = new ParallelAction(run4a5,clawServoRotate13.rotateClawHighBasket(),lift14.liftUp());
        Action scoreOnBasketOne = clawServo12.openClaw();
        Action backFromBasketOne = new ParallelAction(run5back);
        ParallelAction runToSampleTwo = new ParallelAction(run6alt,lift14.liftDown(),clawServoRotate13.rotateClawHome());
        ParallelAction retrieveSampleTwo = new ParallelAction(run7alt,clawServo12.closeClaw());
        ParallelAction runToBasketTwo = new ParallelAction(run8a9, clawServoRotate13.rotateClawHighBasket(),lift14.liftUp());
        Action ScoreOnBasketTwo = clawServo12.openClaw();
        Action BackFromBasketTwo = new ParallelAction(run9altback);


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                        runToHighBar,
                        scoreOnHighBar,
                        runToSampleOne,
                        retrieveSampleOne,
                        runToBasketOne,
                        scoreOnBasketOne,
                        backFromBasketOne,
                        runToSampleTwo,
                        retrieveSampleTwo,
                        runToBasketTwo,
                        ScoreOnBasketTwo,
                        BackFromBasketTwo
                )
        );

    }
}
