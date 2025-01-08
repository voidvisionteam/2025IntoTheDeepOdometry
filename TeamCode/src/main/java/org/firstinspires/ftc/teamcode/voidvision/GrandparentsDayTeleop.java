package org.firstinspires.ftc.teamcode.voidvision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="GrandparentsDayTeleop", group="Pushbot")
public class GrandparentsDayTeleop extends LinearOpMode {
    teenagehwmap robot = new teenagehwmap();
    private ElapsedTime runtime = new ElapsedTime();


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
    boolean rise = false;



    // New flag for left joystick control during the subroutine
    private boolean isLiftMotorRoutineRunning = false;


    //LiftMotorCOnstants
    int initialPosition = 13;
    int thanksgivingTableHeight = 2600;
    double intake = 0;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();
        initialPosition = robot.liftMotor.getCurrentPosition();
        thanksgivingTableHeight = initialPosition+2600;


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


            slowamount = 1*.4;


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

            //robot.liftMotor.setPower(-1*gamepad2.left_stick_y);
            if(rise){
                moveMotorToPosition(robot.liftMotor,thanksgivingTableHeight,.6);
                //robot.liftMotor.setTargetPosition(thanksgivingTableHeight);
            }
            else if (!rise){
                moveMotorToPosition(robot.liftMotor,initialPosition,.4);
                //robot.liftMotor.setTargetPosition(initialPosition);
            }
            else{robot.liftMotor.setPower(0);
                telemetry.addData("lift motor error: either check with Nate, an other programmer, or panic, the current pos is:",robot.liftMotor.getCurrentPosition());
            }
            if(gamepad2.x){clamp = !clamp;sleepWithOpModeCheck(250);}
            if(clamp){robot.clawServo.setPosition(.19);}
            else{robot.clawServo.setPosition(0);}
            if(gamepad2.y){liftClamp = !liftClamp;sleepWithOpModeCheck(250);}
            if(liftClamp){
                rotateClaw();
            }
            else{
                rotateClaw2();
            }
            if(gamepad2.b){rise = !rise; sleepWithOpModeCheck(250);}
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


    private void moveMotorToPosition(DcMotor motor, int targetPosition, double speed) {
        // Set the target position and configure the motor to run to it
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motor.setMode(DcMotor.RunMode.);


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

    private void moveServoToPosition(Servo servo, double targetPosition, double speedFactor) {
        double startPosition = servo.getPosition();


        int steps = 100; // Number of steps for smooth movement
        double delta = (targetPosition - startPosition) / steps;


        for (int i = 0; i < steps; i++) {
            //if (checkForCancel()) return;
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

