package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SkyStone6547Qualifter;

@Autonomous(name = "last year code test")
@Disabled
public class FuckThisImUsingLastYearsCode  extends LinearOpMode {

    SkyStone6547Qualifter bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new SkyStone6547Qualifter(this);

        telemetry.log().add("ready to start");

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("X pos", bot.getRobotPositionX());
            telemetry.addData("Y pos", bot.getRobotPositionY());
            telemetry.update();
        }

        waitForStart();

        bot.DriveToPointPID(24,24,5);

        sleep(1000);
        telemetry.log().add("test");
        bot.DriveToPointPID(48,48,5);
        telemetry.log().add("test 2");

        bot.DriveFieldRealtiveDistance(.4,0,.5);
    }
}
