package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SkyStone6547Qualifter;

@Disabled
public class runQualBot extends LinearOpMode {

    SkyStone6547Qualifter bot = new SkyStone6547Qualifter(this);
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.log().add("Ready to start");
        waitForStart();
    }
}
