package org.firstinspires.ftc.teamcode.voidvision;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="cousinteleopWithLights", group="Pushbot")
public class cousinteleopWithLights extends LinearOpMode {
    cousinhwmapWithLights robot = new cousinhwmapWithLights();
    private ElapsedTime runtime = new ElapsedTime();

    public RevBlinkinLedDriver blinkinLedDriver = null;
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
    int rout3 = 0;
    int orb = 0;
    int inside = 0;
    int home = 0;
    int normal = 0;
    int insidepick = 0;
    int transfer = 0;
    int clampFailSafe=0;
    int transfercount = 0;

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
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
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

            if (gamepad2.left_stick_y >= 0) {
                robot.liftMotor.setPower((-1 * (gamepad2.left_stick_y -0.2)));//-power is up
            }
            else if (gamepad2.left_stick_y< 0){
                robot.liftMotor.setPower((-1 * gamepad2.left_stick_y));
            }

            if(gamepad2.x){
                if (clampFailSafe==0) {
                    clamp = !clamp;
                    sleepWithOpModeCheck(150);
                }
            }
            if(clamp){
                robot.clawServo.setPosition(.1 + robot.clawclaw);
            }
            else{
                robot.clawServo.setPosition(.1);
            }
            //extender
            //hang
            if (gamepad1.a){
                moveServoToPosition(robot.hangServo, robot.hangDown,1);
            }
            if (gamepad1.b){
                moveServoToPosition(robot.hangServo, robot.hangUp,1);
            }
            if (gamepad1.dpad_down){
                robot.hangMotor.setPower(0);
            }
            if (gamepad1.dpad_right){
                robot.hangMotor.setPower(0.3);
            }
            if (gamepad1.dpad_up){
                robot.hangMotor.setPower(0.7);
            }
            //GAMEPAD 2 CONTROLLS

            if(gamepad2.y){
                moveServoToPosition(robot.clawRotateServo,robot.clawRotateHighBasket,1);
                clampFailSafe=0;
            }
            if(gamepad2.b){
                moveServoToPosition(robot.clawRotateServo,robot.clawRotateSpec,1);
                clampFailSafe=0;
            }
            if(gamepad2.a){
                moveServoToPosition(robot.clawRotateServo,robot.clawRotateHome,1);
                clampFailSafe=0;
            }


            robot.range1Servo.setPosition(0+ robot.Finalrange*(gamepad1.left_trigger*0.9));
            robot.range2Servo.setPosition(robot.Finalrange-robot.Finalrange*(gamepad1.left_trigger*0.9));


            //------Organize the while statements------
            if (gamepad2.dpad_down){//home cycler
                if(home==1){
                    home=0;
                }
                else{
                    home=1;
                }

            }
            if (gamepad2.dpad_right){//normal cycler
                if(normal==1){
                    normal=0;
                }
                else{
                    normal=1;
                }
            }
            if (gamepad2.dpad_left){//inside cycler
                if(insidepick==1){
                    insidepick=0;
                }
                else{
                    insidepick=1;
                }
            }
            if (gamepad2.dpad_up){//transfer cycler
                if(transfer==0){
                    transfer=1;
                }
            }

            //--------sub claw -------

            while (home==1) {//home
                if((gamepad1.left_stick_x != 0)||(gamepad1.left_stick_y != 0)){
                    home=0;
                    break;
                }

                if (inside == 0) {
                    moveServoToPosition(robot.subClawServo, robot.subClawClose, 1);
                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    orb = 0;
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmHome);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmHome);
                    moveServoToPosition(robot.subClawPitch, robot.subPitchHome, 1);
                } else if (inside == 1) {

                    moveServoToPosition(robot.subClawServo, robot.subClawInsideGrab, 1);
                    moveServoToPosition(robot.subOrbServo, robot.subOrbPerp, 1);
                    orb = 90;
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmHome);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmHome);
                    moveServoToPosition(robot.subClawPitch, robot.subPitchHome, 1);
                }
                home=0;

            }
            while(normal==1) {//normal
                if((gamepad1.left_stick_x != 0)||(gamepad1.left_stick_y != 0)){
                    normal=0;
                    break;
                }
                if (rout0 == 0) {
                    moveServoToPosition(robot.subClawServo, robot.subClawHalfOpen, 1);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);
                    moveServoToPosition(robot.subClawPitch, robot.subPitchGrab, 1);
                    rout0 = 1;
                    orb = 0;
                    inside = 0;

                } else if (rout0 == 1) {
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmGrab);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmGrab);
                    moveServoToPosition(robot.subClawServo, robot.subClawClose, 1);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);

                    rout0 = 0;
                    rout1 = 0;
                    rout3 = 0;
                    inside = 0;

                }

                normal=0;
            }
            while (insidepick==1) {//inside pickup
                if((gamepad1.left_stick_x != 0)||(gamepad1.left_stick_y != 0)){
                    insidepick=0;
                    break;
                }

                if (rout3 == 0) {
                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    moveServoToPosition(robot.subClawServo, robot.subClawInsidePrep, 1);
                    moveServoToPosition(robot.subClawPitch, robot.subPitchGrab, 1);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);
                    moveServoToPosition(robot.subClawPitch, robot.subPitchGrab, 1);
                    rout0 = 1;
                    orb = 0;
                    rout3 = 1;

                } else if (rout3 == 1) {
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmInsideGrab);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmInsideGrab);
                    sleep(100);
                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    moveServoToPosition(robot.subClawServo, robot.subClawInsideGrab, 1);
                    sleep(100);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);
                    orb = 0;

                    rout0 = 1;
                    rout1 = 0;
                    rout3 = 0;
                    inside = 1;
                }
                insidepick=0;
            }



            while (transfer==1) {//transfer claw to claw
                if((gamepad1.left_stick_x != 0)||(gamepad1.left_stick_y != 0)||(gamepad1.right_stick_x!=0)){
                    transfer=0;
                    break;
                }
                else{

                }
                if (inside == 0) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                    robot.range1Servo.setPosition(0+ robot.Finalrange*0);
                    robot.range2Servo.setPosition(robot.Finalrange-robot.Finalrange*0);
                    moveServoToPosition(robot.subClawPitch, robot.subPitchHome, 1);
                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    orb = 0;
                    robot.clawServo.setPosition(.1);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmHome);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmHome);
                    moveServoToPosition(robot.clawRotateServo, robot.clawRotatePrep, 1);

                    moveServoToPosition(robot.subClawServo, robot.subClawDrop, 1);
                    sleep(100);
                    moveServoToPosition(robot.subClawServo, robot.subClawClose, 1);


                    robot.clawServo.setPosition(.1 + robot.clawclaw);

                    clamp = true;
                    sleep(150);

                    moveServoToPosition(robot.subClawServo, robot.subClawHalfOpen, 1);
                    clampFailSafe=1;
                } else if (inside == 1) {
                    moveServoToPosition(robot.subClawPitch, robot.subPitchGrab,1);
                    orb = 0;
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmHome);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmHome);

                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    moveServoToPosition(robot.subClawServo, robot.subClawInsideGrab, 1);
                }
                transfer=0;


            }
            if (gamepad2.left_bumper) {
                if (orb == 90) {
                    moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                    orb = 0;
                } else if (orb == 0) {
                    moveServoToPosition(robot.subOrbServo, robot.subOrbPerp, 1);
                    orb = 90;

                }
            }
            if (gamepad2.right_bumper) {
                if (inside == 0) {
                    robot.subClawServo.setPosition(robot.subClawHalfOpen);
                }
                else if (inside == 1){
                    robot.subClawServo.setPosition(robot.subClawInsidePrep);
                }
            }








            //telemetry.addData("intake power", robot.intakeServo.getPower());
            //telemetry.addData("claw",robot.clawServo.getPosition());
            //telemetry.addData("RotateClaw",robot.clawRotateServo.getPosition()-robot.FinalrangeClawRotate);
            //telemetry.addData("LIFT",robot.liftMotor.getCurrentPosition());
            //telemetry.addData("LIFTpower",robot.liftMotor.getPower());
            //telemetry.addData("LiftClamp",liftClamp);
            //telemetry.addData("INTAKE",intakePartition);
            //telemetry.addData("CLAMP",clamp);
            //telemetry.addData("clamp",robot.clawServo.getPosition());
            telemetry.addData("LeftTrigger",gamepad2.left_trigger*.3);
            telemetry.update();
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
    private double distanceNumber(){
        robot.init(hardwareMap);
        double distance= robot.distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance: ", distance);
        return distance;
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