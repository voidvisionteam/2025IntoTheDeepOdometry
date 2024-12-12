package org.firstinspires.ftc.teamcode.voidvision.anna;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.voidvision.babyhwmap;

//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name="C3P0", group="Pushbot")
public class testOne extends LinearOpMode {
    babyhwmapAnna robot=new babyhwmapAnna();

    private ElapsedTime runtime = new ElapsedTime();

    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount ;
    static double open = .3;
    static double closed = .5;

    private boolean changed1 = false;

    public void runOpMode(){
        robot.init(hardwareMap);

        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            //Drive

            fwdBackPower = -gamepad1.left_stick_y * slowamount;
            strafePower = -gamepad1.left_stick_x * slowamount;
            turnPower = -gamepad1.right_stick_x * slowamount;

            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);


            robot.leftfrontDrive.setPower(lfPower);
            robot.leftbackDrive.setPower(lbPower);
            robot.rightfrontDrive.setPower(rfPower);
            robot.rightbackDrive.setPower(rbPower);


            if (gamepad1.right_bumper){
                slowamount = 0.5;}
            else if (gamepad1.left_bumper) {
                slowamount = 0.1;}
            else{
                slowamount = 1;}

            //robot.armMotorTwo.setPower(gamepad2.left_stick_y);
            //robot.armMotorOne.setPower(-gamepad2.right_stick_y*.5);



            if(gamepad2.a){
                robot.posServo.setPosition(0);
            } else if(gamepad2.b){
                robot.posServo.setPosition(.75);
                sleep(5000);
            } else{
                robot.posServo.setPosition(0);
            }
            if(gamepad2.x){
                robot.intakeDrive.setPower(1);
            } else if(gamepad2.y){
                robot.intakeDrive.setPower(-1);
            }}
           /* if(gamepad1.a){
                robot.armMotorOne.setPower(1);
            } else if(gamepad1.b){
                robot.armMotorOne.setPower(-1);
            } else{
                robot.armMotorOne.setPower(0);
            }
          if(gamepad1.x){
                robot.armMotorTwo.setPower(1);
            } else if(gamepad1.y){
                robot.armMotorTwo.setPower(-1);

            } else{
              robot.armMotorTwo.setPower(0);
          }*/

           /*while(gamepad1.dpad_up){
                robot.leftfrontDrive.setPower(1);
            }
            while(gamepad1.dpad_left){
                robot.rightfrontDrive.setPower(1);
            }
            while(gamepad1.dpad_down){
                robot.leftbackDrive.setPower(1);
            }
            while(gamepad1.dpad_right){
                robot.rightbackDrive.setPower(1);
            }*/

    }








}