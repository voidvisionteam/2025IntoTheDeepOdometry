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
        while (opModeIsActive()){
            distanceNumber();
        }


    }

    private double distanceNumber(){
        robot.init(hardwareMap);
        double distance= robot.distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance: ", distance);
        return distance;
}
}
/*
           #Apparently it won't allow me to do referencable function because the SDK is too low
           #So I put the one that will probably work down here
           #This is untested

*
* */