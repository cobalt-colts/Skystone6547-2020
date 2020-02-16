package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

public class IntakeColorSensorTest extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547 bot = new DriveTrain6547(this);

        telemetry.addData("Ready to Start", "");
        telemetry.update();

        waitForStart();

        bot.intake(1);
    }
}
