package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

public class testLift extends LinearOpMode {

    public static double liftSpeed=1;
    public static double leeway = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547 bot = new DriveTrain6547(this);

        telemetry.log().add("ready to start");

        while (opModeIsActive())
        {
            bot.updateLift(gamepad1.left_stick_y, liftSpeed, leeway);

            if (bot.a1.onPress())
            {
                bot.zeroEncoder(bot.lift);
                bot.zeroEncoder(bot.liftLeft);
            }
            telemetry.addData("left lift current pos", bot.lift.getCurrentPosition());
            telemetry.addData("right lift current pos", bot.liftLeft.getCurrentPosition());
            telemetry.update();
        }

    }
}
