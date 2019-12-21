package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SkyStone6547Qualifter;

@Autonomous
public class TurnPIDtest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SkyStone6547Qualifter bot = new SkyStone6547Qualifter(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        bot.TurnPID(90,5);
    }
}
