package org.firstinspires.ftc.teamcode.voidvision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp(name="Uncle TOMMY", group="Pushbot")
public class Uncle_tommy extends LinearOpMode {
    cousinhwmapWithLights robot = new cousinhwmapWithLights();
    private ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor distancesensor;



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
    int homecount = 0;
    int homeinsidecount = 0;
    int normalcount = 0;
    int normalroutcount = 0;
    int insidecount = 0;
    int insideroutcount = 0;
    int transfercount=0;
    int transferTime=0;

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
            //hang controls
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
                robot.hangMotor.setPower(0.5);
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

            //robot.subClawPitch.setPosition(gamepad2.left_trigger);

            //robot.subOrbServo.setPosition(gamepad2.left_trigger);
            //robot.clawRotateServo.setPosition(gamepad2.left_trigger*.3);


            robot.range1Servo.setPosition(0+ robot.Finalrange*gamepad1.left_trigger);
            robot.range2Servo.setPosition(robot.Finalrange-robot.Finalrange*gamepad1.left_trigger);
            //if(gamepad1.a){intake = -1d;}
            //else if(gamepad1.x){intake = 1d;}
            //else if(gamepad1.y){intake = 0d;}
            //robot.intakeServo.setPower(intake);
            //robot.transitionServo.setPower(intake);

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
                if(transfer==1){
                    transfer=0;
                }
                else{
                    transfer=1;
                }
            }


            //--------sub claw -------
            //dpad down sets home position
            //dpad right togles between prep and grab-up (orb = 0)
            //dpad left togles between prep and grab up (orb=90)

            if (home==1) {//home
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
            if (normal==1) {//normal

                if (rout0 ==0) {
                    moveServoToPosition(robot.subClawServo, robot.subClawOpen, 1);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);
                    moveServoToPosition(robot.subClawPitch, robot.subPitchGrab, 1);
                    orb = 0;
                    rout0 = 1;
                    inside=0;
                }
                else if (rout0 == 1) {
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmGrab);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmGrab);
                    moveServoToPosition(robot.subClawServo, robot.subClawClose, 1);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);

                    rout0 = 0;
                    rout1 = 0;
                    rout3 = 0;
                    inside = 0;

                    if(distanceNumber() >= 2) {
                        moveServoToPosition(robot.subClawServo, robot.subClawClose, 1);
                        moveServoToPosition(robot.subOrbServo, robot.subOrbHome, 1);
                        orb = 0;
                        robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmHome);
                        robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmHome);
                        moveServoToPosition(robot.subClawPitch, robot.subPitchHome, 1);

                    }
                }

                normal=0;
            }

            while (insidepick==1) {//inside pickup
                if (rout3 == 0) {
                    robot.subOrbServo.setPosition(robot.subOrbHome);
                    robot.subClawServo.setPosition(robot.subClawInsidePrep);
                    robot.subClawPitch.setPosition(robot.subPitchGrab);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);
                    rout0 = 1;
                    orb = 0;
                    rout3 = 1;

                } else if (rout3 == 1) {
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmInsideGrab);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmInsideGrab);
                    sleep(100);
                    robot.subOrbServo.setPosition(robot.subOrbHome);
                    robot.subClawServo.setPosition(robot.subClawInsideGrab);
                    sleep(100);
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmPrep);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmPrep);
                    orb = 0;

                    rout0 = 0;
                    rout1 = 0;
                    rout3 = 0;
                    inside = 1;
                }
                insidepick=0;
            }



            if  (transfer==1) {//transfer claw to claw
                if (inside == 0) {
                    if (transfercount == 0) {
                        robot.range1Servo.setPosition(0 + robot.Finalrange * 0);
                        robot.range2Servo.setPosition(robot.Finalrange - robot.Finalrange * 0);
                        robot.subClawPitch.setPosition(robot.subPitchHome);
                        robot.subOrbServo.setPosition(robot.subOrbHome);
                        orb = 0;
                        transfercount=1;
                    }
                    else if (transfercount == 1) {
                        robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmHome);
                        robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmHome);
                        robot.clawRotateServo.setPosition(robot.clawRotatePrep);
                        robot.subClawServo.setPosition(robot.subClawDrop);
                        robot.clawServo.setPosition(.1);
                        transfercount=2;
                    }
                    else if (transfercount==2) {
                        sleep(400);
                        robot.subClawServo.setPosition(robot.subClawClose);

                        robot.clawServo.setPosition(.1 + robot.clawclaw);

                        clamp = true;
                        transfercount=3;
                    }
                    else if (transfercount == 3) {
                        sleep(150);

                        robot.subClawServo.setPosition(robot.subClawOpen);
                        clampFailSafe = 1;
                        transfercount=0;
                        transfer=0;
                    }
                } else if (inside == 1) {
                    robot.subClawPitch.setPosition(robot.subPitchGrab);
                    orb = 0;
                    robot.basketServo1.setPosition(0 + robot.FinalrangeBasket * robot.swingArmHome);
                    robot.basketServo2.setPosition(robot.FinalrangeBasket - robot.FinalrangeBasket * robot.swingArmHome);

                    robot.subOrbServo.setPosition(robot.subOrbHome);
                    robot.subClawServo.setPosition(robot.subClawInsideGrab);
                    transfer=0;
                }



            }
            if (gamepad2.left_bumper) {
                if (orb == 90) {
                    robot.subOrbServo.setPosition(robot.subOrbHome);
                    orb = 0;
                } else if (orb == 0) {
                    robot.subOrbServo.setPosition(robot.subOrbPerp);
                    orb = 90;

                }
            }
            if (gamepad2.right_bumper) {
                robot.subClawServo.setPosition(robot.subClawOpen);
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
            double distance= robot.distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance: ", distance);
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
    private double distanceNumber(){
        robot.init(hardwareMap);
        double distance= robot.distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance: ", distance);
        return distance;
    }

    private void sleepWithOpModeCheck(long milliseconds) {
        long endTime = System.currentTimeMillis() + milliseconds;
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            // Do nothing, just wait
        }
    }
}