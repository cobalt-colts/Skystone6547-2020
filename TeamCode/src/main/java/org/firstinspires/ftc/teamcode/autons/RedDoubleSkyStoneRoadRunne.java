package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;
import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED double skystone Road Runner")
public class RedDoubleSkyStoneRoadRunne extends LinearOpMode {

    public void runOpMode()
    {
        DriveTrain6547 bot = new DriveTrain6547(this);

        bot.setPoseEstimate(new Pose2d(-(72-bot.getRobotPositionX()), 0,Math.toRadians(180)));

        telemetry.log().add("Ready to start");
        waitForStart();

        Trajectory trajectory = bot.trajectoryBuilder()
                .forward(23)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        bot.followTrajectorySync(trajectory);

        if (bot.isSkystone(bot.colorSensorSideLeft)) //scan stones
        {
            bot.skyStoneLoc = SkyStoneLoc.LEFT;
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .back(2)
            .build());
            telemetry.log().add("LEFT");
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .strafeLeft(5)
                    .build());

        }
        else if (bot.isSkystone(bot.colorSensorSideRight))
        {
            telemetry.log().add("RIGHT");
            bot.skyStoneLoc = SkyStoneLoc.RIGHT;
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(2)
                    .build());
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .strafeRight(5)
                    .build());

        }
        else
        {
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

        }
        bot.intake(1);

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(12)
        .build());

        //sleep(250);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(14)
                .build());
        //sleep(.25);
        bot.turnSync(-90);

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(72)
        .build());

        bot.turnSync(90);
        //setGrabber(1);
        while (opModeIsActive());

        telemetry.log().add("wrote file");

    }
}
