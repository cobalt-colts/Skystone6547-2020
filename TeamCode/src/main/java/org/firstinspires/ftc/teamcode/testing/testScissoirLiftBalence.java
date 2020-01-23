package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

@TeleOp
@Config
public class testScissoirLiftBalence extends LinearOpMode {

    public static int targetSpeed = 200;
    public static int leeway=100;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547 bot = new DriveTrain6547(this);

        telemetry.log().add("ready to start");
        waitForStart();

        bot.zeroEncoder(bot.lift);

        while (opModeIsActive())
        {
            if (bot.a1.onPress())
            {
                bot.zeroEncoder(bot.lift);
            }
            telemetry.addData("target pos abs", Math.abs(bot.getLiftTargetPos()));
            telemetry.addData("right lift pos abs", Math.abs(bot.lift.getCurrentPosition()));
            telemetry.addData("speed", targetSpeed);
            telemetry.addData("leeway", leeway);
            telemetry.update();
        }
        TelemetryPacket packet = new TelemetryPacket();

//        packet.put("target pos", bot.getLiftTargetPos());
//        packet.put("right lift pos", bot.lift.getCurrentPosition());
//        packet.put("left lift pos", bot.liftLeft.getCurrentPosition());
//        packet.put("speed", targetSpeed);
//        packet.put("leeway", leeway);
//        telemetry.update();

    }
}
