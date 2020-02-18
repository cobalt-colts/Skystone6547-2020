package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@TeleOp
@Config
public class foundationGrabberTest extends LinearOpMode {

    public static double fgraber1Pos=0;
    public static double fFrabber2Pos=0;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("wait for start");
        waitForStart();

        while (opModeIsActive())
        {
            bot.fondationGrabber.setPosition(fgraber1Pos);
            bot.fondationGrabber2.setPosition(fFrabber2Pos);
        }
    }
}
