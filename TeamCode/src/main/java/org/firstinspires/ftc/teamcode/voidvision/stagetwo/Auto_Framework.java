package org.firstinspires.ftc.teamcode.voidvision.stagetwo;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
import org.firstinspires.ftc.teamcode.voidvision.Auto_Util;
import org.firstinspires.ftc.teamcode.voidvision.teenagehwmap;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Auto_Framework",group = "Autonomous")
public  class Auto_Framework extends Auto_Util {

    /*Robot Component Classes*/
    public class LightStrip{
        private RevBlinkinLedDriver revBlinkinLedDriver=null;
        public LightStrip(HardwareMap hardwareMap ){
            revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        public LightStrip(HardwareMap hardwareMap, RevBlinkinLedDriver.BlinkinPattern blinkinPattern ){
            revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            revBlinkinLedDriver.setPattern(blinkinPattern);
        }
    }
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
            return new ClawServo.CloseClaw();
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
            return new ClawServo.OpenClaw();
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
            clawservorotate.setPosition(ClawRotateTopBasketPos+.11);
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
            return new RotateClawUpUp();
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
            targetPositionUpperRung = 2318+initialPosition+400-30-30-50; // Adjust based on desired lift distance
            targetpositiontest = targetPositionUpperRung-300-300;
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
    public class ExtendShelf{
        Servo range1Servo,range2Servo;
        double Finalrange = 0.45;
        public ExtendShelf(HardwareMap hardwareMap){
            range1Servo = hardwareMap.get(Servo.class,"hippo1");
            range2Servo = hardwareMap.get(Servo.class,"hippo2");
            range1Servo.setPosition(0);
            range2Servo.setPosition(Finalrange);
        }

    }
    public class IntakeComponent{
        Servo basketServo1,basketServo2;
        double FinalrangeBasket = 0.48;
        public IntakeComponent(HardwareMap hardwareMap){
            basketServo1 = hardwareMap.get(Servo.class,"basket2");
            basketServo2 = hardwareMap.get(Servo.class,"basket1");
            basketServo1.setPosition(0);
            basketServo2.setPosition(FinalrangeBasket);
        }
    }
    public class Flywheels{
        CRServo intakeServo,transitionServo = null;
        public Flywheels(HardwareMap hardwareMap){
            intakeServo = hardwareMap.get(CRServo.class,"intake");
            transitionServo = hardwareMap.get(CRServo.class,"transServo");;
        }
    }
    /*
    * things to use to auto paths:
    * spline to constant heading to get the samples*/

    /*Auto Variables*/
    double fullTurn = 2*Math.PI*1.032;

    Pose2d beginPose = null;
    Pose2d rightBeginPose = new Pose2d(0, 0, 0);
    Pose2d leftBeginPose = new Pose2d(0, 28, 0);

    MecanumDrive drive = null;
    ClawServo clawServo12 = null;
    ClawServoRotate clawServoRotate13 = null;
    Lift lift14 = null;
    LightStrip lightStrip = null;

    Action run1 = null;
    Action run2 = null;
    Action run3 = null;
    Action run4 = null;
    Action run5 = null;
    Action run6 = null;
    Action run7 = null;
    Action run8= null;

    Action runwait,run5back,runTest,runTest2,runwait2,runwait3,run6alt,run7alt,run8alt,run9alt,run9altback,run1a2Right,run1a2Left,run4a5,run8a9 = null;

    public void setupAutoFramework(){
        //Define initial Position
        beginPose = new Pose2d(0, 0, 0);
        //Instantiate Mec Drive
        drive = new MecanumDrive(hardwareMap, beginPose);
        //Init Claw
        clawServo12 = new ClawServo(hardwareMap);
        //Init Claw Lift
        clawServoRotate13 = new ClawServoRotate(hardwareMap);
        //Init Lift
        lift14 = new Lift(hardwareMap);
        lightStrip = new LightStrip(hardwareMap);
    }

    public void setupAutoActionsRight(){
        //Define Actions

        run1 = drive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(21,6))
                .strafeTo(new Vector2d(21,0))
                .strafeTo(new Vector2d(21,6))
                .build();
        run2 = drive.actionBuilder(new Pose2d(21,6,0))
                .strafeTo(new Vector2d(25+.1,6))
                .build();
        run3 = drive.actionBuilder(new Pose2d(25+.1,6,0))
                //.strafeTo(new Vector2d(0,6))
                .strafeTo(new Vector2d(23,4+57))
                .strafeTo(new Vector2d(33,57+4))
                .waitSeconds(.25)
                .build();
        runwait = drive.actionBuilder(new Pose2d(32, 61, 0)).waitSeconds(.5).build();
        run4 = drive.actionBuilder(new Pose2d(32,61,0))
                //.turn((110/360)*fullTurn)
                //.strafeTo(new Vector2d(-10,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .build();
        run5 = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        run5back = drive.actionBuilder(new Pose2d(8,70,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(13,63))
                .build();
        run6 = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .turn((135/360d)*fullTurn)
                .build();
        run7 = drive.actionBuilder(new Pose2d(10,68,(-.5*Math.PI)))
                .strafeTo(new Vector2d(0,-6))
                .build();
        run8 = drive.actionBuilder(new Pose2d(0,-6,(-.5*Math.PI)))
                .strafeTo(new Vector2d(95,0))
                .build();
        runTest = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(10,0))
                .turn(.25*fullTurn)
                .build();
        runTest2 = drive.actionBuilder(beginPose)
                .waitSeconds(.5)
                .build();
        runwait2 = drive.actionBuilder(beginPose)
                .waitSeconds(2)
                .build();
        runwait3 = drive.actionBuilder(beginPose)
                .waitSeconds(.5)
                .build();
        run6alt = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .turn((-135/360d)*fullTurn)
                //.strafeTo(new Vector2d(13,57+5))
                .strafeTo(new Vector2d(13,57+5+9+(5/8d)))
                .strafeTo(new Vector2d(35,57+5+9+(5/8d)))
                .build();
        run7alt = drive.actionBuilder(new Pose2d(35,57+5+9+(5/8d),0)).waitSeconds(.5).build();
        run8alt = drive.actionBuilder(new Pose2d(35,57+5+9+(5/8d),0))
                //.turn((110/360)*fullTurn)
                //.strafeTo(new Vector2d(-10,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .build();
        run9alt = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        run9altback = drive.actionBuilder(new Pose2d(8,70,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(13,63))
                .build();
        run1a2Right = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,0))
                .strafeTo(new Vector2d(25+.1,6))
                .build();
        run1a2Left = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,28-5))
                .strafeTo(new Vector2d(25+.1,28-5))
                .build();
        run4a5= drive.actionBuilder(new Pose2d(32,61,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        run8a9 = drive.actionBuilder(new Pose2d(35,57+5+9+(5/8d),0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
    }
    public void setupAutoActionsLeft(){
        //Define Actions
        run1 = drive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(21,6))
                //.strafeTo(new Vector2d(21,28))
                .strafeTo(new Vector2d(21,28-5))
                .build();
        run2 = drive.actionBuilder(new Pose2d(21,28-5,0))
                .strafeTo(new Vector2d(25+.1,28-5))
                .build();
        run3 = drive.actionBuilder(new Pose2d(25+.1,28-5,0))
                //.strafeTo(new Vector2d(0,6))
                .strafeTo(new Vector2d(23,4+57))
                .strafeTo(new Vector2d(32,57+4))
                .waitSeconds(.25)
                .build();
        runwait = drive.actionBuilder(new Pose2d(32,61,0)).waitSeconds(.5).build();
        run4 = drive.actionBuilder(new Pose2d(32,61,0))
                //.turn((110/360)*fullTurn)
                //.strafeTo(new Vector2d(-10,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .build();
        run5 = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        run5back = drive.actionBuilder(new Pose2d(8,70,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(13,63))
                .build();
        run6 = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .turn((135/360d)*fullTurn)
                .build();
        run7 = drive.actionBuilder(new Pose2d(10,68,(-.5*Math.PI)))
                .strafeTo(new Vector2d(0,-6))
                .build();
        run8 = drive.actionBuilder(new Pose2d(0,-6,(-.5*Math.PI)))
                .strafeTo(new Vector2d(95,0))
                .build();
        runTest = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(10,0))
                .turn(.25*fullTurn)
                .build();
        runTest2 = drive.actionBuilder(beginPose)
                .waitSeconds(.5)
                .build();
        runwait2 = drive.actionBuilder(beginPose)
                .waitSeconds(2)
                .build();
        runwait3 = drive.actionBuilder(beginPose)
                .waitSeconds(.5)
                .build();
        run6alt = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .turn((-135/360d)*fullTurn)
                //.strafeTo(new Vector2d(13,57+5))
                .strafeTo(new Vector2d(13,57+5+9+(5/8d)))
                .strafeTo(new Vector2d(32,57+5+9+(5/8d)))
                .build();
        run7alt = drive.actionBuilder(new Pose2d(32,57+5+9+(5/8d),0)).waitSeconds(.5).build();
        run8alt = drive.actionBuilder(new Pose2d(32,57+5+9+(5/8d),0))
                //.turn((110/360)*fullTurn)
                //.strafeTo(new Vector2d(-10,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .build();
        run9alt = drive.actionBuilder(new Pose2d(13,63,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        run9altback = drive.actionBuilder(new Pose2d(8,70,(135/360d)*(2*Math.PI)))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(13,63))
                .build();
        run1a2Right = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,0))
                .strafeTo(new Vector2d(25+.1,6))
                .build();
        run1a2Left = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(21,28-5))
                .strafeTo(new Vector2d(25+.1,28-5))
                .build();
        run4a5= drive.actionBuilder(new Pose2d(32,61,0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
        run8a9 = drive.actionBuilder(new Pose2d(35,57+5+9+(5/8d),0))
                .strafeTo(new Vector2d(13,63))
                .turn((135/360d)*fullTurn)
                .waitSeconds(.2)
                .strafeTo(new Vector2d(8,70))
                .build();
    }
    public void setupAutoActionsRightEstimate(boolean right){
        if(right){
            run1 = drive.actionBuilder(beginPose)
                    //.strafeTo(new Vector2d(21,6))
                    .strafeTo(new Vector2d(21,0))
                    .strafeTo(new Vector2d(21,6))
                    .build();
            run2 = drive.actionBuilder(new Pose2d(21,6,0))
                    .strafeTo(new Vector2d(25+.1,6))
                    .build();
            run3 = drive.actionBuilder(new Pose2d(25+.1,6,0))
                    //.strafeTo(new Vector2d(0,6))
                    .strafeTo(new Vector2d(23,4+57))
                    .strafeTo(new Vector2d(33,57+4))
                    .waitSeconds(.25)
                    .build();
        }
        else{
            run1 = drive.actionBuilder(beginPose)
                    //.strafeTo(new Vector2d(21,6))
                    //.strafeTo(new Vector2d(21,28))
                    .strafeTo(new Vector2d(21,28-5))
                    .build();
            run2 = drive.actionBuilder(new Pose2d(21,28-5,0))
                    .strafeTo(new Vector2d(25+.1,28-5))
                    .build();
            run3 = drive.actionBuilder(new Pose2d(25+.1,28-5,0))
                    //.strafeTo(new Vector2d(0,6))
                    .strafeTo(new Vector2d(23,4+57))
                    .strafeTo(new Vector2d(32,57+4))
                    .waitSeconds(.25)
                    .build();
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        setupAutoFramework();
        setupAutoActionsRight();
        ParallelAction runToHighBar = new ParallelAction(run1a2Right,clawServoRotate13.rotateClawMid(),lift14.liftUpB());
        Action scoreOnHighBar = lift14.liftDown();
        ParallelAction runToSampleOne = new ParallelAction(run3,clawServoRotate13.rotateClawDown(),clawServo12.openClaw());
        ParallelAction retrieveSampleOne = new ParallelAction(runwait,clawServo12.closeClaw());
        ParallelAction runToBasketOne = new ParallelAction(run4a5,clawServoRotate13.rotateClawUp(),lift14.liftUp());
        Action scoreOnBasketOne = clawServo12.openClaw();
        Action backFromBasketOne = new ParallelAction(run5back);
        ParallelAction runToSampleTwo = new ParallelAction(run6alt,lift14.liftDown(),clawServoRotate13.rotateClawDown());
        ParallelAction retrieveSampleTwo = new ParallelAction(run7alt,clawServo12.closeClaw());
        ParallelAction runToBasketTwo = new ParallelAction(run8a9, clawServoRotate13.rotateClawUp(),lift14.liftUp());
        Action ScoreOnBasketTwo = clawServo12.openClaw();
        Action BackFromBasketTwo = new ParallelAction(run9altback);

        waitForStart();

        /*Actions.runBlocking(new SequentialAction(
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
        );*/
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilde2(beginPose)
                                .setTangent(0)
                                .splineTo(new Vector2d(15,-10),-0.25*fullTurn)
                                //.waitSeconds(1)
                                //.splineTo(new Vector2d(10+20,-10-24),-0.5*fullTurn)
                                //.setTangent(0)
                                //.splineTo(new Vector2d(5+24,-10-24),-.5*fullTurn)
                                //.splineTo(new Vector2d(10,-10-10),-0.55*fullTurn)
                                //.waitSeconds(1)
                                //.splineToSplineHeading(new Pose2d(15,-33.5,-.5*.963*fullTurn),-0.25*(fullTurn))
                                .splineToSplineHeading(new Pose2d(15,-33.5,-1.00000000009*Math.PI),-0.25*(fullTurn))

                                //.waitSeconds(1)
                                //.setTangent(-0.25*(fullTurn))

                                //.splineToSplineHeading(new Pose2d(15,-33.5-1,-.5*.965*fullTurn),-0.25*(fullTurn))
                                //.waitSeconds(1)
                                .strafeTo(new Vector2d(5,-33.5))
                                //.waitSeconds(1)
                                //.splineToSplineHeading(new Pose2d(15,-33.5-9.5,-1*Math.PI),-0.5*(2*Math.PI))
                                .strafeTo(new Vector2d(15,-33.5-9.5))
                                //.waitSeconds(1)
                                .strafeTo(new Vector2d(5,-33.5-9.5))
                                .turnTo(-.5*Math.PI)
                                .turnTo(0-.00000000001)
                                .build()
                )
        );

    }
}
