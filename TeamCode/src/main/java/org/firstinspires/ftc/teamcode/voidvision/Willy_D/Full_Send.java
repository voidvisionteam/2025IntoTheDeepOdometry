package org.firstinspires.ftc.teamcode.voidvision.Willy_D;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "3 spec STATE",group = "Autonomous")
public class Full_Send extends FullSendFramework{

    @Override
    public void runOpMode() throws InterruptedException{
        setupAutoFramework();
        Action runSpecialTestRun = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(25.1,6))
                .strafeTo(new Vector2d(25+.1,-22))
                .strafeTo(new Vector2d(49,-22))
                .strafeTo(new Vector2d(49,-32))
                .strafeTo(new Vector2d(9,-32))
                .strafeTo(new Vector2d(49,-32))
                .strafeTo(new Vector2d(49,-41))
                .strafeTo(new Vector2d(9,-41))
                .strafeTo(new Vector2d(15,-41))

                //.strafeTo(new Vector2d(21,-20))
                //.turn((180/360d)*fullTurn)
                //.strafeTo(new Vector2d(4,-16))

                .waitSeconds(.5)

                .strafeTo(new Vector2d(15,-16))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(26.1-.5,8))//score
                .waitSeconds(.5)

                .strafeTo(new Vector2d(15,-16))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(26.1-.5,8))//score
                .waitSeconds(.5)

                .build();
        Action run1a2Right = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,0))
                .strafeTo(new Vector2d(26+.1,6))
                .build();
        Action runToHighBar = new SequentialAction(
                new ParallelAction(
                        hangServo.hangServoUp(),
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                new ParallelAction(
                        run1a2Right,
                        clawServoRotate13.rotateClawSpec(),
                        lift14.liftUpB()));

        Action scoreOnHighBar = lift14.liftDown();
        Action CollectSpecimensRun = drive.actionBuilde2(new Pose2d(26+.1,6,0))
                .strafeTo(new Vector2d(25+.1,-22))
                .strafeTo(new Vector2d(49,-22))
                .strafeTo(new Vector2d(49,-30))
                .strafeTo(new Vector2d(9,-32))
                .strafeTo(new Vector2d(49,-32))
                .strafeTo(new Vector2d(49,-39))
                .strafeTo(new Vector2d(9,-41))
                .build();
        Action goToSpecimen = drive.actionBuilder(new Pose2d(9,-41,0))
                .strafeTo(specimenLocation1)
                .build();
        Action CollectSpecimens = new SequentialAction(
                new ParallelAction(
                        lift14.liftDown(),
                        clawServoRotate13.rotateClawHome(),
                        new SequentialAction(
                                CollectSpecimensRun,
                                goToSpecimen
                        )
                )
        );


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        runToHighBar,
                        scoreOnHighBar,
                        CollectSpecimens,

                        Grab(specimenLocation),
                        TransferSpecimen(specimenLocation,true,1),
                        moveToGrabSpecimenLocation(new Pose2d(26.1-1,6+2*(1),0)),

                        Grab(specimenLocation),
                        TransferSpecimen(specimenLocation,true,2),
                        Park(new Pose2d(23,5,0))
                )

        );
    }
}
