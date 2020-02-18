package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@Autonomous(name="move lift test", group = "test")
public class moveLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        bot.setLiftTargetPos(bot.lift.getCurrentPosition() + 1000);

        while (opModeIsActive())
        {
            bot.setLiftToTargetPos(bot.getLiftTargetPos(), 50);
            bot.outputTelemetry();
            //bot.moveLift(-1000,50,4);
        }
    }
}
