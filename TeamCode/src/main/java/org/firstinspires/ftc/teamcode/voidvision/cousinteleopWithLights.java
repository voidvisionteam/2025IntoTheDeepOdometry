package org.firstinspires.ftc.teamcode.voidvision;

///Dear Nate, the function you are looking to copy is lightSystem
///---Jette

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="cousinteleopWithLights", group="Pushbot")
public class cousinteleopWithLights extends LinearOpMode {
    cousinhwmapWithLights robot = new cousinhwmapWithLights();
    private ElapsedTime runtime = new ElapsedTime();



    double YellowTargetGreenPercent = 0.31;
    double RedTargetGreenPercent = 0.31;
    double BlueTargetBluePercent = 0.31;

    final double BLUE_MINIMUM = 10;
    final double RED_MINIMUM = 10;
    final double GREEN_MINIMUM = 10;
    String BlinkinColor = "none";

    double colorRed; //= robot.colorSensor.red();
    double colorBlue; // = robot.colorSensor.blue();
    double colorGreen;// = robot.colorSensor.green();
    double colorAlpha;// = robot.colorSensor.alpha();
    double totalLight;// = colorRed + colorBlue + colorGreen;
    double colorRedPercent;
    double colorGreenPercent;
    double colorBluePercent;

    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount = 1;
    static double direction = -1;
    public boolean clamp = false;
    public boolean liftClamp = false;
    private boolean intakePartition = true;


    // Servo sequence control flags
    private boolean isRoutineRunning = false;
    private boolean retract = false;  // boolean for retract mode
    private boolean retract2 = false;

    private boolean UpperRungRunning;
    private boolean LowerRungRunning;
    private boolean NoRungRunning;



    // New flag for left joystick control during the subroutine
    private boolean isLiftMotorRoutineRunning = false;


    //LiftMotorCOnstants
    int initialPosition = 13;
    int targetPositionLowerBasket = 1802; // Adjust based on desired lift distance
    int targetPositionUpperBasket = 2570; // Adjust based on desired lift distance
    int targetPositionLowerRung = 902; // Adjust based on desired lift distance
    int targetPositionUpperRung = 2318; // Adjust based on desired lift distance
    double intake = 0;

    double endgameLiftBoost = 1;


    int rout0 = 0;
    int rout1 = 0;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();
        initialPosition = robot.liftMotor.getCurrentPosition();
        targetPositionLowerBasket = 1802+initialPosition; // Adjust based on desired lift distance
        targetPositionUpperBasket = 2570+initialPosition; // Adjust based on desired lift distance
        targetPositionLowerRung = 902+initialPosition; // Adjust based on desired lift distance
        targetPositionUpperRung = 2318+initialPosition; // Adjust based on desired lift distance


        while (opModeIsActive()) {
            // ---- Drive Control ----
            fwdBackPower = direction * -gamepad1.left_stick_y * slowamount;
            strafePower = direction * -gamepad1.left_stick_x * slowamount;
            turnPower = gamepad1.right_stick_x * slowamount;


            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);


            robot.leftfrontDrive.setPower(lfPower);
            robot.leftbackDrive.setPower(lbPower);
            robot.rightfrontDrive.setPower(rfPower);
            robot.rightbackDrive.setPower(rbPower);


            slowamount = .75;


            // ---- Mecanum Drive Adjustments ----
            if (gamepad1.right_bumper) {
                flipWheelConfigurationBackward();
                telemetry.addData("Direction-Backward:", direction);
            } else {
                flipWheelConfigurationNormal();
                telemetry.addData("Direction-Normal:", direction);
            }


            if (gamepad1.left_bumper) {
                slowamount = .1;
            }
            //if(gamepad1.b){intakePartition = !intakePartition;sleepWithOpModeCheck(200);}

            if(gamepad2.right_bumper){endgameLiftBoost = .9;}
            else{endgameLiftBoost = 1;}

            robot.liftMotor.setPower(-1*gamepad2.left_stick_y*(robot.liftBrake / endgameLiftBoost));

            if(gamepad2.x){clamp = !clamp;sleepWithOpModeCheck(150);

            }
            if(clamp){
                robot.clawServo.setPosition(.1 + robot.clawclaw);
            }
            else{
                robot.clawServo.setPosition(.1);
            }
            //extender
            //moveServosSimultaneously(robot.range1Servo, robot.Finalrange*gamepad2.right_trigger, robot.range2Servo, robot.Finalrange- robot.Finalrange*gamepad2.right_trigger, 0.6);
            if(gamepad2.y){moveServoToPosition(robot.clawRotateServo,robot.ClawRotateTopBasketPos,1);}
            if(gamepad2.b){moveServoToPosition(robot.clawRotateServo,robot.FinalposClawRotate,1);}
            if(gamepad2.a){moveServoToPosition(robot.clawRotateServo,robot.FinalrangeClawRotate,1);}


            robot.range1Servo.setPosition(0+ robot.Finalrange*gamepad2.right_trigger);
            robot.range2Servo.setPosition(robot.Finalrange-robot.Finalrange*gamepad2.right_trigger);
            //Move the Basket Rotationally
            robot.basketServo1.setPosition(0+robot.FinalrangeBasket*gamepad2.left_trigger);
            robot.basketServo2.setPosition(robot.FinalrangeBasket-robot.FinalrangeBasket*gamepad2.left_trigger);
            if(gamepad1.a){intake = -1d;}
            else if(gamepad1.x){intake = 1d;}
            else if(gamepad1.y){intake = 0d;}
            //robot.intakeServo.setPower(intake);
            //robot.transitionServo.setPower(intake);

            //--------sub claw -------
            //dpad down sets home position
            //dpad left togles between prep and grab-up (orb = 0)
            //dpad right togles between prep and grab up (orb=90)
            if(gamepad2.dpad_down){//home
                moveServoToPosition(robot.basketServo1, robot.swingArmHome,1);
                moveServoToPosition(robot.basketServo2, robot.swingArmHome, 1 );
                moveServoToPosition(robot.subClawServo, robot.subClawClose,1);
                moveServoToPosition(robot.subOrbServo, robot.subOrbHome,1);

            }
            if(gamepad2.dpad_left){//pick up normal
                if(rout0==0){
                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    moveServoToPosition(robot.subClawServo, robot.subClawOpen, 1);
                    moveServoToPosition(robot.basketServo2, robot.swingArmPrep, 1);
                    moveServoToPosition(robot.basketServo1, robot.swingArmPrep, 1);
                    rout0=1;
                }
                else if (rout0==1){
                    moveServoToPosition(robot.basketServo2, robot.swingArmGrab, 1);
                    moveServoToPosition(robot.basketServo1, robot.swingArmGrab, 1);
                    sleep(1000);
                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    moveServoToPosition(robot.subClawServo, robot.subClawClose, 1);
                    sleep(1000);
                    moveServoToPosition(robot.basketServo2, robot.swingArmPrep, 1);
                    moveServoToPosition(robot.basketServo1, robot.swingArmPrep, 1);

                    rout0=0;
                }

            }
            if(gamepad2.dpad_right){//pick up 90
                if(rout1==0){
                    moveServoToPosition(robot.subOrbServo, robot.subOrbPerp, 1);
                    moveServoToPosition(robot.subClawServo, robot.subClawOpen, 1);
                    moveServoToPosition(robot.basketServo2, robot.swingArmPrep, 1);
                    moveServoToPosition(robot.basketServo1, robot.swingArmPrep, 1);
                    rout1=1;
                }
                else if (rout1==1){
                    moveServoToPosition(robot.basketServo2, robot.swingArmGrab, 1);
                    moveServoToPosition(robot.basketServo1, robot.swingArmGrab, 1);
                    sleep(1000);
                    moveServoToPosition(robot.subOrbServo, robot.subOrbPerp, 1);
                    moveServoToPosition(robot.subClawServo, robot.subClawClose, 1);
                    sleep(1000);
                    moveServoToPosition(robot.basketServo2, robot.swingArmPrep, 1);
                    moveServoToPosition(robot.basketServo1, robot.swingArmPrep, 1);
                    moveServoToPosition(robot.subOrbServo,robot.subOrbHome, 1);

                    rout1=0;
                }

            }
            if(gamepad2.dpad_up){//transfer claw to claw
                moveServoToPosition(robot.subClawServo, robot.subClawOpen, 1);
                moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                moveServoToPosition(robot.basketServo2, robot.swingArmHome, 1);
                moveServoToPosition(robot.basketServo1, robot.swingArmHome, 1);
                sleep(200);
                moveServoToPosition(robot.subClawServo, robot.subClawOpen, 1);
                robot.clawServo.setPosition(.1);

                //moveServoToPosition(robot.clawRotateServo,robot.transClawRotate,1);
                sleep(200);
                //robot.clawServo.setPosition(.1 + robot.clawClose);
                moveServoToPosition(robot.clawRotateServo,robot.FinalrangeClawRotate,1);



            }






            //telemetry.addData("intake power", robot.intakeServo.getPower());
            //telemetry.addData("claw",robot.clawServo.getPosition());
            //telemetry.addData("RotateClaw",robot.clawRotateServo.getPosition()-robot.FinalrangeClawRotate);
            telemetry.addData("LIFT",robot.liftMotor.getCurrentPosition());
            telemetry.addData("LIFTpower",robot.liftMotor.getPower());
            //telemetry.addData("LiftClamp",liftClamp);
            telemetry.addData("INTAKE",intakePartition);
            telemetry.addData("CLAMP",clamp);
            telemetry.addData("clamp",robot.clawServo.getPosition());
            telemetry.update();

            lightSystem();
        }
    }
    private void lightSystem(){
        colorRed = robot.colorSensor.red();
        colorBlue = robot.colorSensor.blue();
        colorGreen = robot.colorSensor.green();
        colorAlpha = robot.colorSensor.alpha();
        totalLight = colorRed + colorBlue + colorGreen;
        colorRedPercent = colorRed / totalLight;
        colorBluePercent = colorBlue / totalLight;
        colorGreenPercent = colorGreen / totalLight;

        if (colorBlue < BLUE_MINIMUM && colorRed < RED_MINIMUM && colorGreen < GREEN_MINIMUM) {
            BlinkinColor = "none";
        } else if (colorBluePercent > BlueTargetBluePercent) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            BlinkinColor = "Blue";
        } else if (colorGreenPercent > YellowTargetGreenPercent) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            BlinkinColor = "Yellow";
        } else if (colorGreenPercent > RedTargetGreenPercent) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            BlinkinColor = "Red";
        } else {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            BlinkinColor = "none";
        }
    }

    /**
     * Checks if the cancel button (gamepad2.b) is pressed and stops the routine if true.
     * @return true if canceled, false otherwise
     */
    private boolean checkForCancel() {
        /*
        if (gamepad2.right_bumper) {  // Use gamepad2.b as the cancel button
            //telemetry.addData("Sequence", "Canceled by user");
            //telemetry.update();
            // Stop the intake servo immediately
            robot.intakeServo.setPower(0);
            isRoutineRunning = false;
            return true;
        }*/
        return false;
    }
    /**
     * Moves a DC motor to a target position with a specified speed.
     *
     * @param motor The DC motor to control.
     * @param targetPosition The encoder target position to move to.
     * @param speed The motor power (0.0 to 1.0) used to reach the target.
     */
    private void moveMotorToPosition(DcMotor motor, int targetPosition, double speed) {
        // Set the target position and configure the motor to run to it
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set the motor power (speed) while ensuring it’s non-negative
        motor.setPower(Math.abs(speed));


        // Wait until the motor reaches the target position
        while (motor.isBusy()) {
            // You can add telemetry or perform other tasks here while waiting
            //telemetry.addData("Motor Position", motor.getCurrentPosition());
            //telemetry.addData("Target Position", targetPosition);
            //telemetry.update();
        }


        // Stop the motor once the target is reached
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*
     * Helper method to move two servos simultaneously
     * targetPosition1: Target for servo1, targetPosition2: Target for servo2
     * speedFactor should be between 0 and 1, where 1 is 100% speed
     */
    private void moveServosSimultaneously(Servo servo1, double targetPosition1, Servo servo2, double targetPosition2, double speedFactor) {
        double startPosition1 = servo1.getPosition();
        double startPosition2 = servo2.getPosition();


        int steps = 100; // Number of steps for smooth movement
        double delta1 = (targetPosition1 - startPosition1) / steps;
        double delta2 = (targetPosition2 - startPosition2) / steps;


        for (int i = 0; i < steps; i++) {
            if (checkForCancel()) return;
            servo1.setPosition(startPosition1 + (delta1 * i));
            servo2.setPosition(startPosition2 + (delta2 * i));
            sleep((long) (20 * (1 - speedFactor))); // Sleep adjusted based on speed factor
        }


        // Ensure both servos end at their exact target positions
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);
    }
    private void moveServoToPosition(Servo servo, double targetPosition, double speedFactor) {
        double startPosition = servo.getPosition();


        int steps = 100; // Number of steps for smooth movement
        double delta = (targetPosition - startPosition) / steps;


        for (int i = 0; i < steps; i++) {
            if (checkForCancel()) return;
            servo.setPosition(startPosition + (delta * i));
            sleep((long) (20 * (1 - speedFactor))); // Sleep adjusted based on speed factor
        }


        // Ensure the servo ends at the exact target position
        servo.setPosition(targetPosition);
    }
    public void rotateClaw(){
        moveServoToPosition(robot.clawRotateServo,robot.ClawRotateTopBasketPos,1);
    }
    public void rotateClaw2(){
        moveServoToPosition(robot.clawRotateServo,robot.FinalrangeClawRotate,1);
    }
    // ---- Drive Configuration Methods ----
    public void flipWheelConfigurationBackward() {
        direction = 1;
    }


    public void flipWheelConfigurationNormal() {
        direction = -1;
    }


    private void sleepWithOpModeCheck(long milliseconds) {
        long endTime = System.currentTimeMillis() + milliseconds;
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            // Do nothing, just wait
        }
    }
}

