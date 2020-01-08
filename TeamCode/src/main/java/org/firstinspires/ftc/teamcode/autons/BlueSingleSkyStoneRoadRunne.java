package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "Blue double skystone Road Runner")
public class BlueSingleSkyStoneRoadRunne extends LinearOpMode {

    public void runOpMode()
    {
        DriveTrain6547 bot = new DriveTrain6547(this);

        bot.setPoseEstimate(new Pose2d(-35,62,Math.toRadians(270)));

        telemetry.log().add("Ready to start");
        waitForStart();

        Trajectory trajectory = bot.trajectoryBuilder()
                .strafeTo(new Vector2d(-36,40))
                .build();
        waitForStart();

        if (isStopRequested()) return;

        bot.followTrajectorySync(trajectory);

        if (bot.isSkystone(bot.colorSensorSideLeft)) //scan stones
        {
            bot.skyStoneLoc = SkyStoneLoc.LEFT;
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(2)
                    .strafeRight(5)
                    .build());
            telemetry.log().add("LEFT");

        }
        else if (bot.isSkystone(bot.colorSensorSideRight))
        {
            telemetry.log().add("RIGHT");
            bot.skyStoneLoc = SkyStoneLoc.RIGHT;
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(2)
                    .strafeLeft(5)
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

        //sleep(.25);
        //bot.turnSync(-90);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,45,Math.toRadians(180)))
                .build());

        //bot.turnRealtiveSync(180);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(24)
                .build());

//        sleep(1000);
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//        .forward(80)
//                .build());
//
//        bot.turnRealtiveSync(270);

//        bot.followTrajectorySync(bot.trajectoryBuilder()
//        .forward(72)
//        .build());

        //bot.turnSync(90);
        //setGrabber(1);

        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        telemetry.log().add("wrote file");

        sleep(1000);

    }
}
