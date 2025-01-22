package org.firstinspires.ftc.teamcode.voidvision;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * teenagehwmap class defines the hardware mapping for the robot's motors, servos, and sensors.
 * It initializes and configures the drive motors, arm mechanisms, and sensor hardware.
 */
public class cousinhwmapWithLights extends HardwareMapUtil {


    // Declare hardware components
    HardwareMap hwmap = null;
    public DcMotor leftfrontDrive = null;
    public DcMotor rightfrontDrive = null;
    public DcMotor leftbackDrive = null;
    public DcMotor rightbackDrive = null;
    public DcMotor liftMotor = null;
    public DcMotor armMotorTwo = null;
    public CRServo armServo = null;
    public Servo posServo = null;
    public Servo range1Servo = null;  // Servo for left range
    public Servo range2Servo = null;  // Servo for right range
    public Servo basketServo1 = null; // Servo for left basket
    public Servo basketServo2 = null; // Servo for right basket
    public Servo clawServo = null;
    public Servo clawRotateServo = null;
    public CRServo intakeServo = null;
    public CRServo transitionServo = null;
    public ColorSensor colorSensor = null;


    // Default positions for range and basket servos
    public double Finalrange = 0.45;
    //.4 is good
    //.6 is bad
    public double FinalrangeBasket = 0.48;
    public double FinalrangeClawRotate = 0.20;
    public double FinalposClawRotate = .3529+ FinalrangeClawRotate;
    public double ClawRotateTopBasketPos = FinalposClawRotate + .15;

    public double liftBrake = .90;
    double clawclaw = .22;


    public RevBlinkinLedDriver blinkinLedDriver = null;

    //TesterWill

    public Servo subClawServo= null;
    public Servo subOrbServo = null;

    public double swingArmHome = 0;
    public double subClawClose = 0;
    public double swingArmPrep = 0.3;
    public double subClawOpen = 0.30;
    public double subOrbHome = 0;
    public double swingArmGrab = 0.4;
    public double subOrbPerp = 0.5;
    double clawClose = .22;






    /**
     * Initializes all hardware components and sets their initial states.
     *
     * @param ahwMap The HardwareMap from the FTC SDK that maps the hardware configuration.
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        // Initialize drive motors
        leftfrontDrive = HardwareInitMotor("leftFront", true);
        rightfrontDrive = HardwareInitMotor("rightFront", false);
        leftbackDrive = HardwareInitMotor("leftBack", true);
        rightbackDrive = HardwareInitMotor("rightBack", false);


        // Initialize arm motors and servos (commented out if not needed yet)
        liftMotor = HardwareInitMotor("liftMotor", false);
        // armMotorTwo = HardwareInitMotor("arm_2", true);
        // armServo = hwMap.get(CRServo.class, "servo");
        // posServo = hwMap.get(Servo.class, "posServo");


        // Initialize range and basket servos
        range1Servo = HardwareInitServo("hippo1", 0); // Left range servo
        range2Servo = HardwareInitServo("hippo2", Finalrange); // Right range servo
        //intakeServo = HardwareInitCRServo("intake", true); // Intake servo
        basketServo1 = HardwareInitServo("basket2", 0); // Left basket servo
        basketServo2 = HardwareInitServo("basket1", FinalrangeBasket); // Right basket servo
        //these are flipped because Will flipped them
        clawServo = HardwareInitServo("claw",clawclaw+.1);
        clawRotateServo = HardwareInitServo("terminator",FinalrangeClawRotate);
        //transitionServo = HardwareInitCRServo("transServo",true);


        subClawServo = HardwareInitServo("subclaw", subClawClose);// Intake servo
        subOrbServo = HardwareInitServo("subOrb",subOrbHome);



        // Initialize color sensor (commented out if not needed yet)
        //colorSensor = hwMap.get(ColorSensor.class, "color");


        /** Set servo directions */
        //armServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);


        //
        /** Set motor zero power behavior (motors stop when zero power is applied) */
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public HardwareMap getHwmap(){return hwmap;}
}

