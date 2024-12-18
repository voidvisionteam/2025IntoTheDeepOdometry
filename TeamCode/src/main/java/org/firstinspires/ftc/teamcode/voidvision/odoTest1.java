package org.firstinspires.ftc.teamcode.voidvision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        Servo clawservo;
        double closed = .47;
        double opened = 0.0;

        public ClawServo(HardwareMap hardwareMap){
            clawservo = hardwareMap.get(Servo.class,"claw");
            clawservo.setPosition(closed);
            //clawservo.setPosition(0);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                telemetry.addData("Claw Closed",true);telemetry.update();
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
                telemetry.addData("Claw Opened",true);telemetry.update();
                clawservo.setPosition(opened);
                telemetry.addData("ClawPos",clawservo.getPosition());
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
        public void closeClaw2(){
            clawservo.setPosition(closed);
        }

        public void openClaw2() {
            clawservo.setPosition(opened);
        }
    }
    public class ClawServoRotate{

        Servo clawservorotate;

        double FinalrangeClawRotate = 0.25;
        double FinalposClawRotate = .3529+ FinalrangeClawRotate;
        double ClawRotateTopBasketPos = FinalposClawRotate + .1;

        public ClawServoRotate(HardwareMap hardwareMap){
            clawservorotate = hardwareMap.get(Servo.class,"terminator");
            clawservorotate.setPosition(FinalrangeClawRotate);
            //clawservo.setPosition(0);
        }
        public class RotateClawUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(ClawRotateTopBasketPos);
                return false;
            }
        }
        public Action rotateClawUp() {
            return new RotateClawUp();
        }

        public class RotateClawDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(FinalrangeClawRotate);
                return false;
            }
        }
        public Action rotateClawDown() {
            return new RotateClawDown();
        }

        public class RotateClawMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(FinalposClawRotate);
                return false;
            }
        }
        public Action rotateClawMid() {
            return new RotateClawMid();
        }

    }
    public class Lift {
        private DcMotorEx lift;
        int initialPosition = 13;
        int targetPositionLowerBasket = 1802+initialPosition; // Adjust based on desired lift distance
        int targetPositionUpperBasket = 2570+initialPosition; // Adjust based on desired lift distance
        int targetPositionLowerRung = 902+initialPosition; // Adjust based on desired lift distance
        int targetPositionUpperRung = 2318+initialPosition; // Adjust based on desired lift distance

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            initialPosition = lift.getCurrentPosition();
            targetPositionLowerBasket = 1802+initialPosition; // Adjust based on desired lift distance
            targetPositionUpperBasket = 2570+initialPosition; // Adjust based on desired lift distance
            targetPositionLowerRung = 902+initialPosition; // Adjust based on desired lift distance
            targetPositionUpperRung = 2318+initialPosition; // Adjust based on desired lift distance
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.5);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPositionLowerBasket) {
                    return true;
                } else {
                    lift.setPower(.1);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.5);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > initialPosition) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    teenagehwmap robot = new teenagehwmap();
    double quarterTurn = 1.0492* Math.PI / 2;
    double fullTurn = 2*Math.PI*1.032;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //Init Claw
        ClawServo clawServo12 = new ClawServo(hardwareMap);
        ClawServoRotate clawServoRotate13 = new ClawServoRotate(hardwareMap);
        Lift lift14 = new Lift(hardwareMap);
        //Set Robot Initial Position

        Action test1 = drive.actionBuilder(beginPose)
                //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                //.splineTo(new Vector2d(0, 60), Math.PI)
                //.strafeTo(new Vector2d(39+19, 0))
                .waitSeconds(.5)
                .turn(-quarterTurn)

                .waitSeconds(.5)
                .turn(quarterTurn)

                .waitSeconds(.5)
                //.turn(-quarterTurn)

                .waitSeconds(.5)
                //.turn(quarterTurn)
                //.waitSeconds(.5)
                //.strafeTo(new Vector2d(0, 0))
                .strafeTo(new Vector2d(10,0))
                .strafeTo(new Vector2d(0,0))
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
                .build();
        Action test2 = drive.actionBuilder(beginPose).turn(fullTurn).build();
        Action test3 = drive.actionBuilder(beginPose).turn(fullTurn).build();
        Action DecSat15 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0,-(39+25)))
                .build();
        //TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();




            waitForStart();
            //clawServoRotate13.clawservorotate.setPosition(clawServoRotate13.ClawRotateTopBasketPos);

            /*Actions.runBlocking(
                clawServoRotate13.rotateClawMid()

            );
            Actions.runBlocking(
                    test2
            );
            Actions.runBlocking(lift14.liftUp());
            Actions.runBlocking(
                    clawServoRotate13.rotateClawDown()
            );
            Actions.runBlocking(lift14.liftDown());
            Actions.runBlocking(
                    test3
            );*/

            Actions.runBlocking(
                    new SequentialAction(
                            clawServoRotate13.rotateClawMid(),
                            test2,
                            lift14.liftUp(),
                            clawServoRotate13.rotateClawDown(),
                            lift14.liftDown(),
                            test3
                    )
            );
        //Actions.runBlocking(new SequentialAction(DecSat15));
            //clawServoRotate13.clawservorotate.setPosition(clawServoRotate13.FinalrangeClawRotate);

    }
}
