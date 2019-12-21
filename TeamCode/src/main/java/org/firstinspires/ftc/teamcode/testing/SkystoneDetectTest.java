package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

/*
This class is for testing purposes.
The user puts the robot so the color sensors are seeing the left and right stones.
The program then determines if the skyStone is the LEFT, CENTER, or RIGHT stone.

 */
@TeleOp
public class SkystoneDetectTest extends SkyStone6547Meet2 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();
        angleZzeroValue=-90;

        zeroEncoders();

        telemetry.log().add("ready to start");

        waitForStart();

        while (opModeIsActive())
        {
            if (isSkystone(colorSensorSideLeft))
            {
                telemetry.addData("LEFT","");
            }
            else if (isSkystone(colorSensorSideRight))
            {
                telemetry.addData("RIGHT","");
            }
            else
            {
                telemetry.addData("CENTER","");
            }
            telemetry.addData("LEFT RED", colorSensorSideLeft.red());
            telemetry.addData("RIGHT RED", colorSensorSideRight.red());
            telemetry.addData("LEFT GREEN", colorSensorSideLeft.green());
            telemetry.addData("RIGHT GREEN", colorSensorSideRight.green());
            telemetry.addData("LEFT BLUE", colorSensorSideLeft.blue());
            telemetry.addData("RIGHT BLUE", colorSensorSideRight.blue());
            telemetry.addData("LEFT ALPHA", colorSensorSideLeft.alpha());
            telemetry.addData("RIGHT ALPHA",colorSensorSideRight.alpha());
            telemetry.addData("LEFT ARGB", colorSensorSideLeft.argb());
            telemetry.addData("RIGHT ARGB",colorSensorSideRight.argb());
            telemetry.update();
        }

    }
}