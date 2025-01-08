package org.firstinspires.ftc.teamcode.voidvision.juliette;
import androidx.appcompat.widget.ActionBarOverlayLayout;

import org.firstinspires.ftc.teamcode.voidvision.HardwareMapUtil;
import org.firstinspires.ftc.teamcode.voidvision.babyhwmap;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;


@TeleOp(name="DemoLights", group="Pushbot")

public class DemoLights extends LinearOpMode {
    lightsHwmap robot = new lightsHwmap();


    //SENSING VALUES FOR BLOCKS (Change me! Change me!)
    double YellowTargetGreenPercent = 0.4;
    double YellowTargetRedPercent = 0.5;
    double RedTargetGreenPercent = 0.2;
    double RedTargetRedPercent = 0.3;
    double BlueTargetBluePercent = 0.4;

    final double BLUE_MINIMUM = 10;
    final double RED_MINIMUM = 10;
    final double GREEN_MINIMUM = 10;
    String BlinkinColor = "none";

    double colorRed = robot.colorSensor.red();
    double colorBlue = robot.colorSensor.blue();
    double colorGreen = robot.colorSensor.green();
    double colorAlpha = robot.colorSensor.alpha();
    double totalLight = colorRed + colorBlue + colorGreen;
    double colorRedPercent;
    double colorGreenPercent;
    double colorBluePercent;

            /* if ((colorRed > RED_MIN_RED) && (colorGreen < RED_MAX_GREEN)) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if ((colorGreen > YELLOW_MIN_GREEN) && (colorRed > YELLOW_MIN_RED) ) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if ((colorBlue > BLUE_MIN_BLUE)&&(colorRed < BLUE_MAX_RED)&&(colorGreen<BLUE_MAX_GREEN)) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }*/


    //Main Code
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        /*colorRed = robot.colorSensor.red();
        colorBlue = robot.colorSensor.blue();
        colorGreen = robot.colorSensor.green();
        colorAlpha = robot.colorSensor.alpha();
        totalLight = colorRed + colorBlue + colorGreen;
        colorRedPercent = colorRed / totalLight;
        colorBluePercent = colorBlue / totalLight;
        colorGreenPercent = colorGreen / totalLight;

        telemetry.addData("Red:", colorRedPercent);
        telemetry.addData("Blue:", colorBluePercent);
        telemetry.addData("Green:", colorGreenPercent);
        telemetry.addData("Color:", BlinkinColor);
        telemetry.update();*/
        while (opModeIsActive()) {
            colorRed = 0;
            colorBlue = 100;
            colorGreen = 0;
            colorAlpha = 0;
            totalLight = colorRed + colorBlue + colorGreen;
            colorRedPercent = colorRed / totalLight;
            colorBluePercent = colorBlue / totalLight;
            colorGreenPercent = colorGreen / totalLight;

            telemetry.addData("Red:", colorRedPercent);
            telemetry.addData("Blue:", colorBluePercent);
            telemetry.addData("Green:", colorGreenPercent);
            telemetry.addData("Color:", BlinkinColor);
            telemetry.update();
            /* if ((colorRed > RED_MIN_RED) && (colorGreen < RED_MAX_GREEN)) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if ((colorGreen > YELLOW_MIN_GREEN) && (colorRed > YELLOW_MIN_RED) ) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if ((colorBlue > BLUE_MIN_BLUE)&&(colorRed < BLUE_MAX_RED)&&(colorGreen<BLUE_MAX_GREEN)) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }*/

            if (colorBlue < BLUE_MINIMUM && colorRed < RED_MINIMUM && colorGreen < GREEN_MINIMUM) {
                BlinkinColor = "none";
            } else if (colorBluePercent > BlueTargetBluePercent) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                BlinkinColor = "Blue";
            } else if (colorGreenPercent > YellowTargetGreenPercent && colorRed < YellowTargetRedPercent) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                BlinkinColor = "Yellow";
            } else if (colorRedPercent > RedTargetRedPercent) {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                BlinkinColor = "Red";
            } else {
                BlinkinColor = "none";
            }
        }
    }
}