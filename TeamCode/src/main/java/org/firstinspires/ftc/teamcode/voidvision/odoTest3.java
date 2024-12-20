package org.firstinspires.ftc.teamcode.voidvision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
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


@Autonomous(name = "odoTest3",group = "Autonomous")
public  class odoTest3 extends Auto_Util {

    public class ClawServo{

        Servo clawservo;
        double closed = .21+.1;
        double opened = 0+0.1;

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

        double FinalrangeClawRotate = 0.2;
        double FinalposClawRotate = .3529+ FinalrangeClawRotate;
        double ClawRotateTopBasketPos = FinalposClawRotate + .15;

        public ClawServoRotate(HardwareMap hardwareMap){
            clawservorotate = hardwareMap.get(Servo.class,"terminator");
            clawservorotate.setPosition(ClawRotateTopBasketPos);
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

        public class RotateClawUpUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(ClawRotateTopBasketPos+.1);
                return false;
            }
        }
        public Action rotateClawUpUp() {
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
        int targetpositiontest = 0;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            initialPosition = lift.getCurrentPosition();
            targetPositionLowerBasket = 1802+initialPosition; // Adjust based on desired lift distance
            targetPositionUpperBasket = 2570+initialPosition+1550; // Adjust based on desired lift distance
            targetPositionLowerRung = 902+initialPosition; // Adjust based on desired lift distance
            targetPositionUpperRung = 2318+initialPosition+400-30-30; // Adjust based on desired lift distance
            targetpositiontest = targetPositionUpperRung-300;
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.99);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPositionUpperBasket) {
                    return true;
                } else {
                    lift.setPower(.13);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftUpB implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.99);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetpositiontest) {
                    return true;
                } else {
                    lift.setPower(.1);
                    return false;
                }
            }
        }
        public Action liftUpB() {
            return new LiftUpB();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.89);
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
        //Define initial Position
        Pose2d beginPose = new Pose2d(0, 0, 0);
        //Instantiate Mec Drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //Init Claw
        ClawServo clawServo12 = new ClawServo(hardwareMap);
        //Init Claw Lift
        ClawServoRotate clawServoRotate13 = new ClawServoRotate(hardwareMap);
        //Init Lift
        Lift lift14 = new Lift(hardwareMap);

        //Define Actions
        Action run1 = drive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(21,6))
                .strafeTo(new Vector2d(21,0))
                .strafeTo(new Vector2d(21,6))
                .build();
        Action run2 = drive.actionBuilder(new Pose2d(21,6,0))
                .strafeTo(new Vector2d(25+.1,6))
                .build();
        Action run3 = drive.actionBuilder(new Pose2d(25+.1,6,0))
                //.strafeTo(new Vector2d(0,6))
                .strafeTo(new Vector2d(23,4+57))
                .strafeTo(new Vector2d(33,57+4))
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
                .strafeTo(new Vector2d(35,57+5+9+(5/8d)))
                .build();
        Action run7alt = drive.actionBuilder(new Pose2d(35,57+5+9+(5/8d),0)).waitSeconds(.5).build();
        Action run8alt = drive.actionBuilder(new Pose2d(35,57+5+9+(5/8d),0))
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


        //TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();




        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(run1,
                        clawServoRotate13.rotateClawMid(),
                        lift14.liftUpB()),
                //score on high bar

                run2,
                lift14.liftDown(),
                clawServoRotate13.rotateClawDown(),
                clawServo12.openClaw(),
                //grab the sample
                run3,
                clawServo12.closeClaw(),
                runwait,
                clawServoRotate13.rotateClawUp(),
                run4,
                //drop the sample
                lift14.liftUp(),
                run5,
                clawServo12.openClaw(),
                run5back,

                //back for more
                lift14.liftDown(),
                clawServoRotate13.rotateClawDown(),
                run6alt,
                clawServo12.closeClaw()
                ,run7alt,

                clawServoRotate13.rotateClawUp(),
                run8alt,
                //drop the sample (again)
                lift14.liftUp(),
                run9alt,
                clawServo12.openClaw(),
                run9altback

        ));
        /*Actions.runBlocking(
                new SequentialAction(
                        clawServoRotate13.rotateClawUp(),
                        lift14.liftUp(),
                        clawServo12.openClaw(),
                        runwait2,
                        clawServo12.closeClaw(),
                        clawServoRotate13.rotateClawDown(),
                        lift14.liftDown(),
                        runTest2
                )
        );*/


        //Actions.runBlocking(new SequentialAction(DecSat15));
        //clawServoRotate13.clawservorotate.setPosition(clawServoRotate13.FinalrangeClawRotate);

    }
}
