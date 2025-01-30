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
import org.firstinspires.ftc.teamcode.voidvision.redRightAutoSPECIAL;
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

        double clawRotateHome=.13;
        double clawRotateSpec=.188;
        double clawRotateHighBasket=.21;
        double clawRotatePrep=.082;
        double clawRotateInit=.14;

        public ClawServoRotate(HardwareMap hardwareMap){
            clawservorotate = hardwareMap.get(Servo.class,"terminator");
            clawservorotate.setPosition(clawRotatePrep);
            //clawservo.setPosition(0);
        }

        public class RotateClawUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(ClawRotateTopBasketPos);
                return false;
            }
        }
        public Action rotateClawUp() {return new RotateClawUp();}

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

        public class RotateClawHome implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(clawRotateHome);
                return false;
            }
        }
        public Action rotateClawHome() {return new RotateClawHome();}

        public class RotateClawSpec implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(clawRotateSpec);
                return false;
            }
        }
        public Action rotateClawSpec() {return new RotateClawSpec();}

        public class RotateClawHighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(clawRotateHighBasket);
                return false;
            }
        }
        public Action rotateClawHighBasket() {return new RotateClawHighBasket();}

        public class RotateClawPrep implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawservorotate.setPosition(clawRotatePrep);
                return false;
            }
        }
        public Action rotateClawPrep() {return new RotateClawPrep();}


    }
    public class Lift {
        private DcMotorEx lift;
        int initialPosition = 13;
        int targetPositionLowerBasket = 1802+initialPosition; // Adjust based on desired lift distance
        int targetPositionUpperBasket = 2570+initialPosition; // Adjust based on desired lift distance
        int targetPositionLowerRung = 902+initialPosition; // Adjust based on desired lift distance
        int targetPositionUpperRung = 2318+initialPosition; // Adjust based on desired lift distance
        int targetpositiontest = 0;
        int targetSpecialGrab = initialPosition + 100;

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

        public class LiftUpSpecialHeight implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.99);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetSpecialGrab) {
                    return true;
                } else {
                    lift.setPower(.1);
                    return false;
                }
            }
        }
        public Action liftUpSpecialHeight() {
            return new LiftUpSpecialHeight();
        }

        public class LiftDownLowBar implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.89);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > targetPositionLowerRung) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDownLowBar(){
            return new LiftDownLowBar();
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
        double sample1=0;
        double sample2=0;
        double sample3=0;

        public ExtendShelf(HardwareMap hardwareMap){
            range1Servo = hardwareMap.get(Servo.class,"hippo1");
            range2Servo = hardwareMap.get(Servo.class,"hippo2");
            range1Servo.setPosition(0);
            range2Servo.setPosition(Finalrange);
        }

        public class extend1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                range1Servo.setPosition(0+Finalrange*sample1);
                range2Servo.setPosition(Finalrange-Finalrange*sample1);
                return false;
            }
        }
        public Action Extend1() {
            return new extend1();
        }

        public class extend2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                range1Servo.setPosition(0+Finalrange*sample2);
                range2Servo.setPosition(Finalrange-Finalrange*sample2);
                return false;
            }
        }
        public Action Extend2() {
            return new extend2();
        }

        public class extend3 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                range1Servo.setPosition(0+Finalrange*sample3);
                range2Servo.setPosition(Finalrange-Finalrange*sample3);
                return false;
            }
        }
        public Action Extend3() {
            return new extend3();
        }


    }
    public class BackIntakeComponent{
        Servo basketServo1,basketServo2;
        double FinalrangeBasket = 0.48;
        double swingArmHome = .025;
        double swingArmPrep = 0.83;
        double swingArmGrab = 0.96;
        double swingArmInsideGrab = 0.97;
        public BackIntakeComponent(HardwareMap hardwareMap){
            basketServo1 = hardwareMap.get(Servo.class,"basket1");
            basketServo2 = hardwareMap.get(Servo.class,"basket2");
            basketServo1.setPosition(0+FinalrangeBasket*swingArmHome);
            basketServo2.setPosition(FinalrangeBasket-FinalrangeBasket*swingArmHome);
        }

        public class swingHome implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                basketServo1.setPosition(0+FinalrangeBasket*swingArmHome);
                basketServo2.setPosition(FinalrangeBasket-FinalrangeBasket*swingArmHome);
                return false;
            }
        }
        public Action SwingHome() {
            return new swingHome();
        }

        public class swingPrep implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                basketServo1.setPosition(0+FinalrangeBasket*swingArmPrep);
                basketServo2.setPosition(FinalrangeBasket-FinalrangeBasket*swingArmPrep);
                return false;
            }
        }
        public Action SwingPrep() {
            return new swingPrep();
        }

        public class swingGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                basketServo1.setPosition(0+FinalrangeBasket*swingArmGrab);
                basketServo2.setPosition(FinalrangeBasket-FinalrangeBasket*swingArmGrab);
                return false;
            }
        }
        public Action SwingGrab() {
            return new swingGrab();
        }

        public class swingInsideGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                basketServo1.setPosition(0+FinalrangeBasket*swingArmInsideGrab);
                basketServo2.setPosition(FinalrangeBasket-FinalrangeBasket*swingArmInsideGrab);
                return false;
            }
        }
        public Action SwingInsideGrab() {
            return new swingInsideGrab();
        }
    }
    public class Flywheels{
        CRServo intakeServo,transitionServo = null;
        public Flywheels(HardwareMap hardwareMap){
            intakeServo = hardwareMap.get(CRServo.class,"intake");
            transitionServo = hardwareMap.get(CRServo.class,"transServo");;
        }
    }
    public class Intake2{

        Servo subClawServo,subOrbServo,subClawPitch;

        double subClawOpen = 0.000;
        double subClawInsidePrep = 0.32;
        double subClawInsideGrab = 0.15;
        double subClawClose = .259;

        double subOrbHome = .9255;
        double subOrbPerp = 0.603;

        double subPitchGrab = .847;
        double subPitchHome = .32;
        double subClawDrop=0.14;

        public Intake2(HardwareMap hardwareMap){
            subClawServo = hardwareMap.get(Servo.class,"subclaw");
            subOrbServo = hardwareMap.get(Servo.class,"subOrb");
            subClawPitch = hardwareMap.get(Servo.class,"subClawPitch");
            subClawServo.setPosition(subClawOpen);
            subOrbServo.setPosition(subOrbHome);
            subClawPitch.setPosition(subPitchHome);

        }
        public class swingHome implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //subClawServo.setPosition(subClawOpen);
                subOrbServo.setPosition(subOrbHome);
                subClawPitch.setPosition(subPitchHome);
                return false;
            }
        }
        public Action SwingHome() {
            return new swingHome();
        }

        public class swingGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subClawServo.setPosition(subClawOpen);
                subOrbServo.setPosition(subOrbHome);
                subClawPitch.setPosition(subPitchGrab);
                return false;
            }
        }
        public Action SwingGrab() {
            return new swingGrab();
        }

        public class closeSubClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subClawServo.setPosition(subClawClose);
                return false;
            }
        }
        public Action CloseSubClaw() {
            return new closeSubClaw();
        }

        public class openSubClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subClawServo.setPosition(subClawOpen);
                return false;
            }
        }
        public Action OpenSubClaw() {
            return new openSubClaw();
        }

        public class dropSubClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subClawServo.setPosition(subClawDrop);
                return false;
            }
        }
        public Action DropSubClaw() {
            return new dropSubClaw();
        }

        public class moveSubOrbHome implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subOrbServo.setPosition(subOrbHome);
                return false;
            }
        }
        public Action MoveSubOrbHome() {
            return new moveSubOrbHome();
        }

        public class moveSubOrbPerp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subOrbServo.setPosition(subOrbPerp);
                return false;
            }
        }
        public Action MoveSubOrbPerp() {
            return new moveSubOrbPerp();
        }
    }
    /*
    * things to use to auto paths:
    * spline to constant heading to get the samples*/

    /*Auto Variables*/
    double fullTurn = 2*Math.PI*1.032;

    public Pose2d beginPose = null;
    Pose2d rightBeginPose = new Pose2d(0, 0, 0);
    Pose2d leftBeginPose = new Pose2d(0, 28, 0);
    Pose2d specimenLocation = new Pose2d(15,-41,0);
    Vector2d specimenLocation1 = new Vector2d(15,-41);

    MecanumDrive drive = null;
    ClawServo clawServo12 = null;
    ClawServoRotate clawServoRotate13 = null;
    Lift lift14 = null;
    LightStrip lightStrip = null;
    BackIntakeComponent backIntakeComponent = null;
    Intake2 intake2 = null;
    ExtendShelf extendShelf = null;


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
        //Init intake2
        intake2 = new Intake2(hardwareMap);
        //Init arms
        backIntakeComponent = new BackIntakeComponent(hardwareMap);
        //Init Lightstrip
        lightStrip = new LightStrip(hardwareMap);
        //Init Shelf
        extendShelf = new ExtendShelf(hardwareMap);
    }
    public void setupAutoFramework(RevBlinkinLedDriver.BlinkinPattern blinkinPattern){
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
        //Init intake2
        intake2 = new Intake2(hardwareMap);
        //Init arms
        backIntakeComponent = new BackIntakeComponent(hardwareMap);
        //Init Lightstrip
        lightStrip = new LightStrip(hardwareMap,blinkinPattern);
        //Init Shelf
        extendShelf = new ExtendShelf(hardwareMap);
    }
    public SequentialAction Grab(Pose2d pose2d) {
        return new SequentialAction(
                //GRAB SetUp
                drive.actionBuilder(pose2d).waitSeconds(.1).build(),
                //HandOffSetupFront
                new ParallelAction(clawServo12.openClaw(),clawServoRotate13.rotateClawHome()),
                //GRAB SetUp
                new ParallelAction(backIntakeComponent.SwingGrab(),intake2.SwingGrab()),
                drive.actionBuilder(pose2d).waitSeconds(1).build(),
                //GRAB
                intake2.CloseSubClaw(),
                drive.actionBuilder(pose2d).waitSeconds(.2).build(),
                //HandOffSetUpBack
                new ParallelAction(backIntakeComponent.SwingHome(),intake2.SwingHome()),
                drive.actionBuilder(pose2d).waitSeconds(.3).build(),
                //intake2.DropSubClaw(),
                new ParallelAction(
                        new SequentialAction(
                                intake2.DropSubClaw(),
                                drive.actionBuilder(pose2d).waitSeconds(.5).build(),
                                intake2.CloseSubClaw()
                        ),
                        clawServoRotate13.rotateClawPrep()),
                //clawServoRotate13.rotateClawPrep(),
                //intake2.CloseSubClaw(),
                drive.actionBuilder(pose2d).waitSeconds(.1).build()

        );
    }
    public SequentialAction Transfer(Pose2d pose2d,Boolean oneEightyDegrees) {
        return new SequentialAction(
                //HandOff
                drive.actionBuilder(pose2d).waitSeconds(.2).build(),
                clawServo12.closeClaw(),
                //drive.actionBuilder(pose2d).waitSeconds(.5).build(),
                intake2.OpenSubClaw(),
                //LIFT!
                drive.actionBuilder(pose2d).waitSeconds(.3).build(),
                new ParallelAction(
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                //SCORE!!
                new ParallelAction(
                        lift14.liftUp(),
                        clawServoRotate13.rotateClawHighBasket(),
                        drive.actionBuilde3(pose2d)
                                //.strafeTo(new Vector2d (8,38))
                                .strafeTo(new Vector2d (8-3+1,38+2+2))
                                .turn((-45/360d)*fullTurn)
                                .build()
                ),
                drive.actionBuilder(pose2d).waitSeconds(.01).build()

        );
    }
    public SequentialAction TransferAlt(Pose2d pose2d,Boolean twoTenDegrees) {
        return new SequentialAction(
                //HandOff
                drive.actionBuilder(pose2d).waitSeconds(.2).build(),
                clawServo12.closeClaw(),
                //drive.actionBuilder(pose2d).waitSeconds(.5).build(),
                intake2.OpenSubClaw(),
                //LIFT!
                drive.actionBuilder(pose2d).waitSeconds(.3).build(),
                new ParallelAction(
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                //SCORE!!
                new ParallelAction(
                        lift14.liftUp(),
                        clawServoRotate13.rotateClawHighBasket(),
                        drive.actionBuilder(pose2d)
                                //.strafeTo(new Vector2d (8,38))
                                .strafeTo(new Vector2d (8-3+1,38+2+2))
                                .turn((-(45+30)/360d)*fullTurn)
                                .build()
                ),
                drive.actionBuilder(pose2d).waitSeconds(.01).build()

        );
    }
    public SequentialAction TransferSpecimen(Pose2d pose2d,Boolean zeroDegrees,int specimenNumber) {
        return new SequentialAction(
                //HandOff
                drive.actionBuilder(pose2d).waitSeconds(.2).build(),
                clawServo12.closeClaw(),
                drive.actionBuilder(pose2d).waitSeconds(.5).build(),
                intake2.OpenSubClaw(),
                //LIFT!
                drive.actionBuilder(pose2d).waitSeconds(.3).build(),
                new ParallelAction(
                        lift14.liftUpSpecialHeight(),
                        clawServoRotate13.rotateClawHome()
                ),
                //SCORE!!
                new ParallelAction(
                        lift14.liftUp(),
                        clawServoRotate13.rotateClawSpec(),
                        drive.actionBuilder(pose2d)
                                .strafeTo(new Vector2d(26+.1,6+2*(specimenNumber)))
                                .build()
                ),
                lift14.liftDownLowBar(),
                drive.actionBuilder(pose2d).waitSeconds(.1).build()

        );
    }
    public Action moveToGrabSpecimenLocation(Pose2d pose2d){
        return new ParallelAction(
                lift14.liftDown(),
                drive.actionBuilder(pose2d)
                        .strafeTo(specimenLocation1)
                        .build()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //setupAutoFramework();
        lightStrip = new LightStrip(hardwareMap,RevBlinkinLedDriver.BlinkinPattern.GREEN);
        //setupAutoActionsRight();

        /*ParallelAction runToHighBar = new ParallelAction(run1a2Right,clawServoRotate13.rotateClawMid(),lift14.liftUpB());
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


        Action sideRoute01 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(26.1,6))
                .strafeTo(new Vector2d(25+.1,-22))
                .strafeTo(new Vector2d(49,-22))
                .strafeTo(new Vector2d(49,-32))
                .strafeTo(new Vector2d(9,-32))
                .strafeTo(new Vector2d(49,-32))
                .strafeTo(new Vector2d(49,-41))
                .strafeTo(new Vector2d(9,-41))
                .strafeTo(new Vector2d(15,-41))
                .build();
        Action sideRoute02 = drive.actionBuilder(new Pose2d(15,-41,0))
                //.strafeTo(new Vector2d(15,-20))
                //.turn((180/360d)*fullTurn)
                .strafeTo(new Vector2d(21,-20))
                .build();
        Action sideRoute03 = drive.actionBuilder(new Pose2d(21,-20,0))
                .waitSeconds(.5)
                .build();
        Action sideRoute04 = drive.actionBuilder(new Pose2d(2,-20,0))
                .strafeTo(new Vector2d(15,-20))
                //.turn((180/360d)*fullTurn*-1)
                .build();
        Action sideRoute05 = drive.actionBuilder(new Pose2d(15,-20,0))
                .strafeTo(new Vector2d(15,6+2))
                .strafeTo(new Vector2d(26.1-.5,6+2))
                .build();
        Action sideRoute040 = drive.actionBuilder(new Pose2d(2,-20,0))
                .splineTo(new Vector2d(25.1-5,8.0),0)
                .build();



        Action sideRoute06 = drive.actionBuilde3(new Pose2d(25.1-.5,6+2,0))
                .strafeTo(new Vector2d(21,-20))
                //.turn((180/360d)*fullTurn)
                //.strafeTo(new Vector2d(21,-20))
                .build();
        Action sideRoute07 = drive.actionBuilder(new Pose2d(21,-20,0))
                .waitSeconds(.5)
                .build();
        Action sideRoute08 = drive.actionBuilder(new Pose2d(2,-20,0))
                .strafeTo(new Vector2d(15,-20))
                //.turn((180/360d)*fullTurn*-1)
                .build();
        Action sideRoute09 = drive.actionBuilder(new Pose2d(15,-20,0))
                .strafeTo(new Vector2d(15,6+4))
                .strafeTo(new Vector2d(26.1,6+4))
                .build();

        Action throwAway = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(1,0))
                .waitSeconds(5)
                .build();
        Action throwAway2 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(1,0))
                .waitSeconds(7)
                .build();

        Action SpecialPart1 = new ParallelAction(sideRoute01,clawServoRotate13.rotateClawDown());

        Action SpecialPart2 = new ParallelAction(sideRoute02,clawServo12.openClaw(),lift14.liftUpSpecialHeight(),clawServoRotate13.rotateClawMid());
        Action SpecialPart3 = new ParallelAction(sideRoute03,new SequentialAction(clawServo12.closeClaw()));
        Action SpecialPart4 = new SequentialAction(lift14.liftUpSpecialHeight(),sideRoute04);
        Action SpecialPart5 = new ParallelAction(sideRoute05,clawServoRotate13.rotateClawMid(),lift14.liftUpB());
        Action scoreSpecialOnBar1 = new ParallelAction(lift14.liftDownLowBar());


        Action SpecialPart6 = new ParallelAction(sideRoute06,new SequentialAction(lift14.liftDown(),clawServo12.openClaw(),lift14.liftUpSpecialHeight()));
        Action SpecialPart7 = new ParallelAction(sideRoute07,new SequentialAction(clawServo12.closeClaw()));
        Action SpecialPart8 = new SequentialAction(lift14.liftUpSpecialHeight(),sideRoute08);
        Action SpecialPart9 = new ParallelAction(sideRoute09,clawServoRotate13.rotateClawMid(),lift14.liftUpB());
        Action scoreSpecialOnBar2 = new ParallelAction(lift14.liftDownLowBar());

        Action runSpecialTestRun = drive.actionBuilder(new Pose2d(26+.1,6,0))
                .strafeTo(new Vector2d(25+.1,-22))
                .strafeTo(new Vector2d(49,-22))
                .strafeTo(new Vector2d(49,-32))
                .strafeTo(new Vector2d(9,-32))
                .strafeTo(new Vector2d(49,-32))
                .strafeTo(new Vector2d(49,-41))
                .strafeTo(new Vector2d(9,-41))
                .strafeTo(new Vector2d(15,-41))

                .strafeTo(new Vector2d(21,-20))
                //.turn((180/360d)*fullTurn)
                //.strafeTo(new Vector2d(4,-16))

                .waitSeconds(.5)

                .strafeTo(new Vector2d(15,-16))
                .turn((180/360d)*fullTurn*-1)

                .strafeTo(new Vector2d(26.1-.5,8))
                .build();*/

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
        );

        Actions.runBlocking(
                /*new SequentialAction(
                        runToHighBar,
                        scoreOnHighBar,
                        SpecialPart1,
                        SpecialPart2,
                        SpecialPart3,
                        SpecialPart4,
                        SpecialPart5,
                        scoreSpecialOnBar1,
                        SpecialPart6,
                        SpecialPart7,
                        SpecialPart8,
                        SpecialPart9,
                        scoreSpecialOnBar2)
        new SequentialAction(
                Grab(beginPose),
                Transfer(beginPose,true)
        )
        );
        */


    }
}
