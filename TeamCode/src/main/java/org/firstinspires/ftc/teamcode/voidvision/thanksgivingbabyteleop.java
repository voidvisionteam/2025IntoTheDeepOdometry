package org.firstinspires.ftc.teamcode.voidvision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="thanksgivingbabyteleop", group="Pushbot")
public class thanksgivingbabyteleop extends LinearOpMode {


    // Robot hardware map instance
    //babyhwmap robot = new babyhwmap();
    thanksgivingteenagehwmap robot= new thanksgivingteenagehwmap();


    // Timer for tracking elapsed time
    private ElapsedTime runtime = new ElapsedTime();


    // Variables for driving and controlling motors
    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount;
    static double open = .3;
    static double closed = .5;
    static double direction = -1;  // Direction modifier for forward/reverse
    static double rangeServoDirection = .01;  // Servo position for range extension


    // Flag to track state changes
    private boolean changed1 = false;


    @Override
    public void runOpMode() {
        // Initialize the robot hardware map
        robot.init(hardwareMap);


        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the driver to start the teleop
        waitForStart();


        // Main control loop while the OpMode is active
        while (opModeIsActive()) {


            // Drive control based on gamepad input
            fwdBackPower = -direction * -gamepad1.left_stick_y * slowamount;
            strafePower = -direction * -gamepad1.left_stick_x * slowamount;
            turnPower = gamepad1.right_stick_x * slowamount;


            // Calculating power for each wheel
            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);


            // Setting motor powers for mecanum drive
            robot.leftfrontDrive.setPower(lfPower);
            robot.leftbackDrive.setPower(lbPower);
            robot.rightfrontDrive.setPower(rfPower);
            robot.rightbackDrive.setPower(rbPower);


            // Setting speed modifier for slow mode
            slowamount = 0.5;


            // Flip wheel configuration when right bumper is pressed
            if (gamepad1.right_bumper) {
                flipWheelConfigurationBackward();
                telemetry.addData("Direction", "Backward");
            } else {
                flipWheelConfigurationNormal();
                telemetry.addData("Direction", "Normal");
            }


            // Adjust slow mode speed when left bumper is pressed
            if (gamepad1.left_bumper) {
                slowamount = 0.25;
            } else {
                slowamount = 0.6;
            }
            //moveServosSimultaneously(robot.basketServo1, 0+robot.FinalrangeBasket*.25+.75*robot.FinalrangeBasket*gamepad1.right_trigger, robot.basketServo2, robot.FinalrangeBasket-robot.FinalrangeBasket*.25-.75*robot.FinalrangeBasket*gamepad1.right_trigger, 0.6);
            robot.basketServo1.setPosition(0+robot.FinalrangeBasket*.25+.75*robot.FinalrangeBasket*gamepad1.right_trigger);
            robot.basketServo2.setPosition(robot.FinalrangeBasket-robot.FinalrangeBasket*.25-.75*robot.FinalrangeBasket*gamepad1.right_trigger);

            telemetry.update();
        }
    }


    /**
     * Flips the wheel configuration for backward driving.
     * Forward becomes backward, strafe directions also swap.
     */
    public void flipWheelConfigurationBackward() {
        direction = -1;
    }


    /**
     * Resets the wheel configuration to normal driving.
     */
    public void flipWheelConfigurationNormal() {
        direction = 1;
    }


    /**
     * Displays the value of the right trigger for debugging.
     */
    public void testTriggerRight() {
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
    }


    /**
     * Displays the value of the left trigger for debugging.
     */
    public void testTriggerLeft() {
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
    }


    /**
     * Gradually increases the servo's position by multiplying it.
     */
    public void extendRangeServoDirection() {
        rangeServoDirection *= 1.05;
    }
    private void moveServosSimultaneously(Servo servo1, double targetPosition1, Servo servo2, double targetPosition2, double speedFactor) {
        double startPosition1 = servo1.getPosition();
        double startPosition2 = servo2.getPosition();


        int steps = 100; // Number of steps for smooth movement
        double delta1 = (targetPosition1 - startPosition1) / steps;
        double delta2 = (targetPosition2 - startPosition2) / steps;


        for (int i = 0; i < steps; i++) {
            //if (checkForCancel()) return;
            servo1.setPosition(startPosition1 + (delta1 * i));
            servo2.setPosition(startPosition2 + (delta2 * i));
            sleep((long) (20 * (1 - speedFactor))); // Sleep adjusted based on speed factor
        }


        // Ensure both servos end at their exact target positions
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);
    }
}

