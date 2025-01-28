package org.firstinspires.ftc.teamcode.voidvision.stagetwo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
                .strafeTo(new Vector2d (8,38))
                .waitSeconds(2)
                .turn((135/360d)*fullTurn)
                .strafeTo(new Vector2d (8-3,38+2))
                .waitSeconds(2)
                .turn((45/360d)*fullTurn)
                .strafeTo(new Vector2d (8,38))


                .waitSeconds(2)
                .strafeTo(new Vector2d (8,34))
                .waitSeconds(2)

                .strafeTo(new Vector2d (8,38))
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
        waitForStart();
        Actions.runBlocking(
                runSpecialTestRun
        );
    }
}
