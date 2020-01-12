package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "Blue Double Skystone Road Runner", group = "auton")
public class BlueDoubleSkyStoneRoadRunner extends LinearOpMode {

    public void runOpMode()
    {
        DriveTrain6547 bot = new DriveTrain6547(this);

        bot.setPoseEstimate(new Pose2d(-35,62,Math.toRadians(270)));

        telemetry.log().add("Ready to start");
        waitForStart();

        Trajectory trajectory = bot.trajectoryBuilder()
                .strafeTo(new Vector2d(-36,36))
                .build();
        waitForStart();

        if (isStopRequested()) return;

        bot.followTrajectorySync(trajectory);

        if (bot.isSkystone(bot.colorSensorSideLeft)) //scan stones
        {
            bot.skyStoneLoc = SkyStoneLoc.LEFT;
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(2)
                    .strafeLeft(15)
                    .build());
            telemetry.log().add("LEFT");

        }
        else if (bot.isSkystone(bot.colorSensorSideRight))
        {
            telemetry.log().add("RIGHT");
            bot.skyStoneLoc = SkyStoneLoc.RIGHT;
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(2)
                    .strafeRight(12)
                    .build());
        }
        else
        {
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

        }
        bot.outtake(1);
        sleep(500);
        bot.intake(1);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(24)
                .build());

        //sleep(250);

        //sleep(.25);


        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,45,Math.toRadians(180)))
                .build());


        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(12)
                .build());

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(14)
        .build());

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .splineTo(new Pose2d(-30,41, Math.toRadians(180)))
                .build());

        if (bot.skyStoneLoc == SkyStoneLoc.RIGHT)
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(20)
                    .build());
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .strafeLeft(22)
                    .build());
        }
        else if (bot.skyStoneLoc == SkyStoneLoc.CENTER)
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(13)
                    .build());

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .strafeLeft(22)
                    .build());
        }
        else if (bot.skyStoneLoc == SkyStoneLoc.LEFT)
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(5)
                    .build());

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .strafeLeft(22)
                    .build());
        }
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(8)
                .build());

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-15,40, Math.toRadians(180)))
                .build());


        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(28)
                .build());

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(14)
                .build());


//        sleep(1000);
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//        .forward(80)
//                .build());
//

//        bot.followTrajectorySync(bot.trajectoryBuilder()
//        .forward(72)
//        .build());

        //setGrabber(1);

        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        telemetry.log().add("wrote file");

    }
}
