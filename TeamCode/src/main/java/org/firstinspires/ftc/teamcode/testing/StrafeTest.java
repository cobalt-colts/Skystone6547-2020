package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInQualifier.SkyStone6547Qualifter;

@Autonomous
@Disabled
public class StrafeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SkyStone6547Qualifter bot = new SkyStone6547Qualifter(this);

        telemetry.log().add("ready to start");

        waitForStart();

        bot.strafeToDistanceYPID(24,5);
    }
}
