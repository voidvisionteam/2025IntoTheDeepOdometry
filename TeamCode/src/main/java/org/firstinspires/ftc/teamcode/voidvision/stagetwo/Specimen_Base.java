package org.firstinspires.ftc.teamcode.voidvision.stagetwo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "Spec",group = "Autonomous")
public class Specimen_Base extends Auto_Framework{

    @Override
    public void runOpMode() throws InterruptedException{
        //Define initial Position
        beginPose = new Pose2d(0, 0, 0);
        //Instantiate Mec Drive
        drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
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
        Actions.runBlocking(
                runSpecialTestRun
        );
    }
}
