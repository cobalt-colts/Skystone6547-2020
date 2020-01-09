package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

@Autonomous
@Disabled
public class RoadRunnerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547 bot = new DriveTrain6547(this);

        bot.setPoseEstimate(new Pose2d(-40,-60,90));
        telemetry.log().add("Ready to Start");

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder()
            .strafeTo(new Vector2d(-20,-40))
                .lineTo(new Vector2d(-40,-60))
                .build());

    }
}
