package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
    Color sensor template by 6547 Cobalt Colts

    In our experience, this detects the skystone correctly every time.

 */
public class ColorSensorTemplete extends LinearOpMode {

    //Color sensors
    public ColorSensor colorSensorLeft;
    public ColorSensor colorSensorRight;

    @Override
    public void runOpMode() throws InterruptedException {

        //int sensors
        colorSensorLeft = hardwareMap.get(ColorSensor.class,"colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive())
        {
            if (isSkyStone(colorSensorLeft)) //does the left color sensor detect a skystone?
            {
                telemetry.addData("Skystone Location", "LEFT");
            }
            else if (isSkyStone(colorSensorRight)) //the left color sensor didn't detect a skystone, but does the right one?
            {
                telemetry.addData("Skystone Location","RIGHT");
            }
            else //both sensors didn't detect a skystone, therefore the skystone must be in the center
            {
                telemetry.addData("Skystone Location", "CENTER");
            }
            telemetry.update();
        }

    }
    /*
    determine if the color sensor selected is seeing a skystone
    returns true if it sees a skystone
    returns false if it sees a regular stone
     */
    public boolean isSkyStone(ColorSensor colorSensor)
    {
            return (colorSensor.red()*colorSensor.green()) / Math.pow(colorSensor.blue(),2) < 3;
    }
}
