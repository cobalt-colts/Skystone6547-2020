package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@Autonomous
@Disabled
public class splineTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        bot.setRunLift(true);

        bot.setPoseEstimate(new Pose2d(-36,-24,Math.toRadians(90)));

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder(false)
                //  .reverse()
                .lineTo(new Vector2d(-10,-40))
                .lineToSplineHeading(new Vector2d(0,-40),Math.toRadians(180))
                //.lineToSplineHeading(new Vector2d())
//                .splineTo(new Pose2d(35,-40,Math.toRadians(180)))
//                .splineTo(new Pose2d(50,-24,Math.toRadians(270)))
                .build());
    }
}
