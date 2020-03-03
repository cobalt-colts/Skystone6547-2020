package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@TeleOp(name = "Calibrate Horizontal Servo Slide", group = "cali")
@Config
@Disabled
public class CalibrateSLider extends LinearOpMode {

    public static double slideSpeed = .0004;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("ready to start");
        waitForStart();

        while (opModeIsActive())
        {
            bot.updateGamepads();
           // if (bot.rightBumper1.isPressed()) bot.updateServo(bot.grabberSlide, 1, slideSpeed, .95, 0.05);
            //if (bot.leftBumper1.isPressed()) bot.updateServo(bot.grabberSlide, -1, slideSpeed, .95, 0.05);

           // if (bot.a1.onPress()) bot.writeFile(bot.GRABBER_MIN_FILE_NAME, bot.grabberSlide.getPosition());
           // if (bot.b1.onPress()) bot.writeFile(bot.GRABBER_MAX_FILE_NAME, bot.grabberSlide.getPosition());
            telemetry.addData("A to set MIN, B to set MAX", "");
           // telemetry.addData("grabber pos", bot.grabberSlide.getPosition());
            telemetry.update();
        }
    }
}
