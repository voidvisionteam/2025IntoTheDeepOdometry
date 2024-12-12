package org.firstinspires.ftc.teamcode.voidvision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "odoTest1",group = "Autonomous")
public  class odoTest1 extends Auto_Util {
    public class ClawServo{
        private Servo clawservo;
        private double closed = .19;
        private double opened = .0;
        public ClawServo(HardwareMap hardwareMap){
            clawservo = hardwareMap.get(Servo.class,"claw");
            clawservo.setPosition(closed);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservo.setPosition(closed);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservo.setPosition(opened);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    teenagehwmap robot = new teenagehwmap();
    double quarterTurn = 1.0492* Math.PI / 2;
    @Override
    public void runOpMode() throws InterruptedException {
        //Init Claw
        ClawServo clawServo = new ClawServo(hardwareMap);
        //Set Robot Initial Position
        Pose2d beginPose = new Pose2d(0, 0, 0);


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                    drive.actionBuilder(beginPose)
                            //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                            //.splineTo(new Vector2d(0, 60), Math.PI)
                            //.strafeTo(new Vector2d(39+19, 0))
                            .waitSeconds(.5)
                            .turn(-quarterTurn)

                            .waitSeconds(.5)
                            .turn(quarterTurn)

                            .waitSeconds(.5)
                            .turn(-quarterTurn)

                            .waitSeconds(.5)
                            .turn(quarterTurn)
                            //.waitSeconds(.5)
                            //.strafeTo(new Vector2d(0, 0))
                            //.waitSeconds(.5)
                            //.turn(-quarterTurn)

                            //.waitSeconds(.5)
                            //.turn(quarterTurn)

                            //.waitSeconds(.5)
                            //.turn(-quarterTurn)

                            //.waitSeconds(.5)
                            //.turn(quarterTurn)
                            //.waitSeconds(.5)
                            //.setTangent(0)
                            //.splineToSplineHeading( new Pose2d(10, -10, quarterTurn), Math.PI / 2)
                            //.waitSeconds(.5)
                            //.splineToSplineHeading( new Pose2d(0, 0, quarterTurn), Math.PI / 2)

                            .build(),
                            clawServo.openClaw(),
                            clawServo.closeClaw()
                    )
            );
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
