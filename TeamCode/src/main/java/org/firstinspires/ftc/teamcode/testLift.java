package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

@TeleOp(name = "test scissor lift calibrate")
@Config
@Disabled
public class testLift extends LinearOpMode {

    public static double liftSpeed=100;
    public static double leeway = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547 bot = new DriveTrain6547(this);

        bot.zeroEncoder(bot.liftLeft);
        bot.zeroEncoder(bot.lift);

        telemetry.log().add("ready to start");
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("gamepad1 left stick", gamepad1.left_stick_y);
            bot.updateLift(gamepad1.left_stick_y, liftSpeed, leeway);

            if (bot.a1.onPress())
            {
                bot.zeroEncoder(bot.lift);
                bot.zeroEncoder(bot.liftLeft);
            }
            telemetry.addData("right lift current pos", bot.lift.getCurrentPosition());
            telemetry.addData("left lift current pos", bot.liftLeft.getCurrentPosition());
            telemetry.addData("right lift pow", bot.lift.getPower());
            telemetry.addData("left lift pow", bot.liftLeft.getPower());
            telemetry.update();
        }

    }
}
