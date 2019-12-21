package org.firstinspires.ftc.teamcode.testing.MuiltthreadTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SkyStone6547Qualifter;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

/**
 * Created by Drew from 11874 on 12/19/2019.
 */
@Autonomous
@Disabled
public class testCompletableFuture extends LinearOpMode {

    SkyStone6547Qualifter bot;

    @Override
    public void runOpMode() throws InterruptedException {

        bot = new SkyStone6547Qualifter(this);

        telemetry.log().add("ready to start");

        waitForStart();

        CompletableFuture<Void> leftBack = CompletableFuture.runAsync(() ->
        {
            try {
                bot.DriveFieldRealtiveDistance(.4, 0, 2);
            }
            catch (Exception e)
            {
                telemetry.log().add(e.getMessage());
            }
        });;

        sleep(1000);

        CompletableFuture<Void> everything = CompletableFuture.allOf(leftBack);

        telemetry.log().add("started");

        while (!everything.isDone() && opModeIsActive())
        {
            bot.outputTelemetry();
        }

    }
}
