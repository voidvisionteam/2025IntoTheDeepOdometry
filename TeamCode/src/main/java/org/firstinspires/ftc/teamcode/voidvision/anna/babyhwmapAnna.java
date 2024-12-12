package org.firstinspires.ftc.teamcode.voidvision.anna;




import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.voidvision.HardwareMapUtil;

//import org.firstinspires.ftc.teamcode.voidvision.HardwareMapUtil2;

/**
 * This class is responsible for mapping and initializing the hardware components of the robot.
 * It extends the HardwareMapUtil to utilize methods like HardwareInitMotor and HardwareInitServo.
 */
public class babyhwmapAnna extends HardwareMapUtil2 {


    // Reference to the robot's hardware map
    HardwareMap hwmap = null;


    // Declaring motor variables
    public DcMotor leftfrontDrive = null;
    public DcMotor rightfrontDrive = null;
    public DcMotor leftbackDrive = null;
    public DcMotor rightbackDrive = null;


    // Declaring arm motor and servo variables
    public DcMotor armMotorOne = null;
    public DcMotor armMotorTwo = null;
    public DcMotor intakeDrive = null;
    public CRServo armServo = null;
    public Servo posServo = null;
    public Servo servo = null;
    public Servo clawServo = null;
    public Servo clawRotateServo = null;

    /**
     * Initializes the hardware components of the robot.
     * @param ahwMap Reference to the hardware map passed from the OpMode.
     */
    public void init(HardwareMap ahwMap) {
        // Assign the hardware map reference
        hwMap = ahwMap;


        // Initialize drive motors with proper direction
        leftfrontDrive = HardwareInitMotor("leftFront", true);
        rightfrontDrive = HardwareInitMotor("rightFront", false);
        leftbackDrive = HardwareInitMotor("leftBack", true);
        rightbackDrive = HardwareInitMotor("rightBack", false);


        // The following components are currently commented out and can be enabled if needed
        //servo = HardwareInitServo("servo", 0);
        intakeDrive= HardwareInitMotor("intakeDrive", true);
        //armMotorTwo = HardwareInitMotor("arm_2", true);
        //armServo = hwMap.get(CRServo.class, "servo");
        posServo = HardwareInitServo("posServo",0);
        //servo = hwmap.get(Servo.class, "servo");


        // Set motor behaviors to brake when power is zero
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Uncomment if arm motors are used
        //armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

