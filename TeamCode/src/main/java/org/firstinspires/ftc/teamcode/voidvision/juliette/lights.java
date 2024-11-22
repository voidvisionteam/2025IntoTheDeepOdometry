package org.firstinspires.ftc.teamcode.voidvision.juliette;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.voidvision.teenagehwmap;

@TeleOp(name="lights", group="Pushbot")
public class lights extends LinearOpMode {
    teenagehwmap robot = new teenagehwmap();
    private ElapsedTime runtime = new ElapsedTime();
    //Variables for Red vs yellow vs blue blocks
    final double RED_MIN_RED = 150;
    final double RED_MAX_GREEN = 40;
    final double BLUE_MIN_BLUE = 150;
    final double YELLOW_MIN_RED = 80;
    final double YELLOW_MIN_GREEN = 80;

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

        RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        while (opModeIsActive()) {
            //---- Color Sensor Update ----
            //int colorRed = colorSensor.red();
            //int colorBlue = colorSensor.blue();
            //int colorGreen = colorSensor.green();
            //int colorAlpha = colorSensor.alpha();
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


            slowamount = 1;


            // ---- Mecanum Drive Adjustments ----
            if (gamepad1.right_bumper) {
                flipWheelConfigurationBackward();
                telemetry.addData("Direction-Backward:", direction);
            } else {
                flipWheelConfigurationNormal();
                telemetry.addData("Direction-Normal:", direction);
            }


            if (gamepad1.left_bumper) {
                slowamount = .25;
            }
            //if(gamepad1.b){intakePartition = !intakePartition;sleepWithOpModeCheck(200);}
            if(!intakePartition){
                robot.liftMotor.setPower(-1*gamepad2.left_stick_y);
                if(gamepad2.x){clamp = !clamp;sleepWithOpModeCheck(250);}
                if(clamp){robot.clawServo.setPosition(.19);}
                else{robot.clawServo.setPosition(0);}
                //extender
                moveServosSimultaneously(robot.range1Servo, robot.Finalrange*gamepad2.right_trigger, robot.range2Servo, robot.Finalrange- robot.Finalrange*gamepad2.right_trigger, 0.6);
                if(gamepad2.y){liftClamp = !liftClamp;sleepWithOpModeCheck(250);}
                if(liftClamp){
                    rotateClaw();
                }
                else{
                    rotateClaw2();
                }


                if(gamepad2.dpad_right){
                    // Step 1: Move the motor up by 3000 encoder counts at full speed


                    //moveMotorToPosition(robot.liftMotor, targetPositionUpperRung,.8);
                    //robot.liftMotor.setPower(0);
                    //hi
                }
                else if(gamepad2.dpad_left){

                    // Step 1: Move the motor up by 3000 encoder counts at full speed


                    //moveMotorToPosition(robot.liftMotor, targetPositionLowerRung,.8);
                    //robot.liftMotor.setPower(0);
                }
                else if(gamepad2.dpad_down){
                    // Step 1: Move the motor up by 3000 encoder counts at full speed


                    //moveMotorToPosition(robot.liftMotor, initialPosition,.8);
                    //robot.liftMotor.setPower(0);
                }
                if(gamepad2.right_bumper){intakePartition = true;}
            }
            else if(intakePartition){

                //----PUT COLOR LIGHTS SHOW WHICH BLOCK HERE---
                /*
                if((colorRed>RED_MIN_RED)&&(colorBlue>BLUE_MIN_BLUE)&&(colorGreen>YELLOW_MIN_GREEN)){
                   blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
                else if ((colorRed>RED_MIN_RED)&&(colorGreen<RED_MAX_GREEN)){
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                else if ((colorGreen>YELLOW_MIN_GREEN)&&(colorRed>YELLOW_MIN_RED)){
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }
                else if (colorBlue>BLUE_MIN_BLUE){
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
                else {
                   blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
                 */



                // ---- Second Servo Subroutine (Triggered by gamepad2.b) ----
                moveServosSimultaneously(robot.range1Servo,0+ robot.Finalrange*gamepad2.right_trigger, robot.range2Servo, robot.Finalrange-robot.Finalrange*gamepad2.right_trigger, 1);
                if (gamepad2.b && !isRoutineRunning) {
                    isRoutineRunning = true;
                    // // Adjust the power as needed
                    new Thread(() -> runSecondServoSequence()).start();  // Execute the servo sequence in a separate thread
                    //robot.intakeServo.setPower(0); // Adjust the power as needed
                }
                if(gamepad1.a){intake = -1d;}
                else if(gamepad1.x){intake = 1d;}
                else if(gamepad1.y){intake = 0d;}
                robot.intakeServo.setPower(intake);
                if(gamepad2.right_bumper){intakePartition = false;}
            }

            //telemetry.addData("intake power", robot.intakeServo.getPower());
            //telemetry.addData("claw",robot.clawServo.getPosition());
            //telemetry.addData("RotateClaw",robot.clawRotateServo.getPosition()-robot.FinalrangeClawRotate);
            telemetry.addData("LIFT",robot.liftMotor.getCurrentPosition());
            //telemetry.addData("LiftClamp",liftClamp);
            telemetry.addData("INTAKE",intakePartition);
            telemetry.update();
        }
    }
    /*
     * Second Autonomous Servo Sequence
     * Moves servos based on retract flag triggered by gamepad2.b
     */
    private void runSecondServoSequence() {
        //telemetry.addData("Second Servo Sequence", "Started");
        telemetry.update();
        //robot.intakeServo.setPower(-1.0);


        //moveServosSimultaneously(robot.basketServo1, 0, robot.basketServo2, robot.FinalrangeBasket, 0.99);
        //moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, robot.Finalrange, 0.6);
        /**
         moveMultipleServosWithSpeeds(
         new Servo[] { robot.range1Servo, robot.range2Servo, robot.basketServo1, robot.basketServo2 },
         new double[] { 0, robot.Finalrange, 0, robot.FinalrangeBasket },
         new double[] { 0.6, 0.6, 0.7, 0.7 }
         );
         **/


        if (checkForCancel()) {isRoutineRunning = false;return;}
        moveServosSimultaneously(robot.basketServo1, robot.FinalrangeBasket*.95, robot.basketServo2, robot.FinalrangeBasket- robot.FinalrangeBasket*.95, 1);
        if (checkForCancel()) {isRoutineRunning = false;return;}


        while (!gamepad2.left_bumper && opModeIsActive()) {
            //robot.intakeServo.setPower(-1.0);
            //telemetry.addData("Waiting for Left Bumper", "Press gamepad2.left_bumper to retract");
            //telemetry.update();
            if (checkForCancel()) return;
        }


        retract = true;


        if (retract) {
            //robot.intakeServo.setPower(0);
            // Step 4: Move basketServo1 and basketServo2 to retract positions
            moveServosSimultaneously(robot.basketServo1, 0 + robot.FinalrangeBasket * 0.75, robot.basketServo2, robot.FinalrangeBasket * 0.25, 1);
            if (checkForCancel()) {isRoutineRunning = false;return;}
            sleepWithOpModeCheck(500);


            // Step 5: Move range1Servo and range2Servo to final positions
            //moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, robot.Finalrange, 0.6);
            if (checkForCancel()) {isRoutineRunning = false;return;}


            // Step 6: Return basketServo1 and basketServo2 to starting positions


            //robot.range1Servo.setPosition(0);
            //robot.range2Servo.setPosition(robot.Finalrange);


            if (checkForCancel()) {isRoutineRunning = false;return;}


        }


        // Wait for gamepad2.left_bumper to be pressed to set retract to true
        while (!gamepad2.left_bumper && opModeIsActive()) {
            //robot.intakeServo.setPower(-1.0);
            //telemetry.addData("Waiting for Left Bumper", "Press gamepad2.left_bumper to retract");
            //telemetry.update();
            if (checkForCancel()) return;
        }
        retract2 = true;
        if(retract2){
            //robot.intakeServo.setPower(-1.0);
            moveServosSimultaneously(robot.basketServo1, 0, robot.basketServo2, robot.FinalrangeBasket, 1);
            if (checkForCancel()) {isRoutineRunning = false;return;}
        }


        sleepWithOpModeCheck(1000);


        // Stop the intake servo when the second servo sequence ends
        //robot.intakeServo.setPower(0); // Stop the intake servo


        isRoutineRunning = false;
        telemetry.addData("Second Servo Sequence", "Completed");
        telemetry.update();
    }
    private void runUpperRungSequence(){}
    private void runLowerRungSequence(){}
    private void runNoRungSequence(){}

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
        moveServoToPosition(robot.clawRotateServo,robot.FinalposClawRotate,1);
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


