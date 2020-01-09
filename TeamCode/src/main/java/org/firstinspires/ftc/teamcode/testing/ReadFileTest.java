package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

@Disabled
public class ReadFileTest extends SkyStone6547 {

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

            telemetry.addData("raw angle", angles.firstAngle);
            telemetry.addData("calibrated angle", getIMUAngle());
            telemetry.update();
        }
    }
}
