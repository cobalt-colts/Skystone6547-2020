package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@TeleOp(name = "Calibrate Horizontal Servo Slide State", group = "cali")
@Config
public class CalibrateSLiderState extends LinearOpMode {

    public static double slideSpeed = .0004;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("ready to start");
        waitForStart();

        telemetry.clear();
        telemetry.addData("Move the Slide off the Servo","");
        telemetry.addData("Push A when done","");
        telemetry.update();

        while (!gamepad1.a)
        {
            //wait for a to be pressed
        }

        bot.grabberSlide.setPosition(.5);
        telemetry.clear();
        telemetry.addData("Moving Servo","");
        telemetry.addData("Just wait a bit","");
        telemetry.update();

        sleep(1000);

        telemetry.clear();
        telemetry.addData("Move the slide so it starts touching the gear","");
        telemetry.addData("BUT NOT ANY FURTHER","");
        telemetry.addData("Push A when done","");
        telemetry.update();

        while (!gamepad1.a)
        {
            //wait for a to be pressed
        }

        double grabberMax = bot.grabberSlide.getPosition();
        double grabberMin = grabberMax-.215;

        bot.writeFile(bot.GRABBER_MIN_FILE_NAME,grabberMin);
        bot.writeFile(bot.GRABBER_MAX_FILE_NAME,grabberMax);

        telemetry.clear();
        telemetry.addData("Servo Calibrated Successfully.  Now testing","");

        bot.grabberSlide.setPosition(grabberMax);
        telemetry.clear();
        telemetry.addData("Moving to MAX","");
        telemetry.update();

        sleep(2000);

        bot.grabberSlide.setPosition(grabberMin);
        telemetry.clear();
        telemetry.addData("Moving to MIN","");
        telemetry.update();

        sleep(2000);

    }
}
