package org.firstinspires.ftc.teamcode.testing.stateMachine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SkyStone6547Qualifter;

@Autonomous
@Disabled
public class stateTaste extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SkyStone6547Qualifter bot = new SkyStone6547Qualifter(this);

        telemetry.log().add("ready to start");
        waitForStart();

        newStateMachie machie = new newStateMachie(bot);

        while (opModeIsActive())
        {
            machie.doState();
        }
    }
}
