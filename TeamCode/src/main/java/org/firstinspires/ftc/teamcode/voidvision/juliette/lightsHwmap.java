package org.firstinspires.ftc.teamcode.voidvision.juliette;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.voidvision.HardwareMapUtil;

/**
 * teenagehwmap class defines the hardware mapping for the robot's motors, servos, and sensors.
 * It initializes and configures the drive motors, arm mechanisms, and sensor hardware.
 */
public class lightsHwmap extends HardwareMapUtil {


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
    public ColorSensor colorSensor = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;


    // Default positions for range and basket servos
    public double Finalrange = 0.45;
    //.4 is good
    //.6 is bad
    public double FinalrangeBasket = 0.48;
    public double FinalrangeClawRotate = 0.25;
    public double FinalposClawRotate = .3529+ FinalrangeClawRotate;




    /**
     * Initializes all hardware components and sets their initial states.
     *
     * @param ahwMap The HardwareMap from the FTC SDK that maps the hardware configuration.
     */
    public void init(HardwareMap ahwMap) {
        HardwareMap hwMap = null;
        hwMap = ahwMap;
        //colorSensor = hwMap.get(ColorSensor.class, "color");
        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");

        DistanceSensor distanceSensor = hwMap.get(DistanceSensor.class, "distance");


    }
}

