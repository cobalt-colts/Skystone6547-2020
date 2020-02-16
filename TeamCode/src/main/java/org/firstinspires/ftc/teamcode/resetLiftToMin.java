package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

@TeleOp(group = "calibrate")
public class resetLiftToMin extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547 bot = new DriveTrain6547(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive())
        {
            bot.moveLift(0,50);
        }
    }
}
