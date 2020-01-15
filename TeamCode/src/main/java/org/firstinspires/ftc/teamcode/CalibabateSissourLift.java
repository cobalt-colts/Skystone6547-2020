package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

@TeleOp
public class CalibabateSissourLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547 bot = new DriveTrain6547(this);

        telemetry.log().add("ready to start");
        waitForStart();
        telemetry.log().add("Use the left stick and the right stick to move the lift");

        while (opModeIsActive())
        {
            bot.lift.setPower(gamepad1.right_stick_y);
            bot.liftLeft.setPower(gamepad1.left_stick_y);
        }
    }
}
