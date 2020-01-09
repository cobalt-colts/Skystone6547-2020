package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

/**
 * Created by Drew from 11874 on 10/26/2019.
 */
@Disabled
public class ReadWriteTest extends SkyStone6547Meet2 {

    @Override
    public void runOpMode() {

        init();

        initIMU();

        angleZzeroValue = 0;

        telemetry.log().add("ready to start");
        telemetry.log().add("push A to write file");

        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                writeFile("zeroValue.json", getIMUAngle());
                telemetry.log().add("wrote " + getIMUAngle() + " degrees in zeroValue.json");
            }

            telemetry.addData("raw angle", getIMUAngle()+angleZzeroValue);
            telemetry.addData("calibrated angle", getIMUAngle());
        }
    }
}
