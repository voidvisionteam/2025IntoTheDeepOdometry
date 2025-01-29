package org.firstinspires.ftc.teamcode.voidvision.juliette;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Distance Sensor", group="Pushbot")

public class distanceSensorFunction extends LinearOpMode {
    lightsHwmap robot = new lightsHwmap();

    //Main Code
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        while (opModeIsActive()){
            distance();
        }
    }
//This is the function that you can copy into your code. It requires distance Sensor to be in hardware map.
    //---------------------------------------------------------------------------------------------------------

    public double distance(){
        robot.init(hardwareMap); //instantiates hardwaremap inside the function
        double distanceCm= robot.distanceSensor.getDistance(DistanceUnit.CM); //assigns the distance detected in centimeters.
        telemetry.addData("Distance: ", distanceCm); //shows distance in the control hub
        return distanceCm; //puts distance as the value of the function
    }

//-------------------------------------------------------------------------------------------------------------
}

