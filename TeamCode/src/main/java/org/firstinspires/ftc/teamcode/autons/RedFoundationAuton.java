package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;
import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;


/*
    This auton pulls the foundation to the build site and parks under the skybridge for the RED side
 */
@Autonomous(name = "RED Foundation Auton road runner", group = "auton")
public class RedFoundationAuton extends LinearOpMode
{

    public void runOpMode()
    {
        DriveTrain6547 bot = new DriveTrain6547(this);

        bot.setPoseEstimate(new Pose2d(35,-62,Math.toRadians(270)));
        bot.setAngleZzeroValue(180);
        telemetry.log().add("ready to start");

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
        .splineTo(new Pose2d(40,-34,Math.toRadians(270)))
                .build());

        sleep(500);

        bot.setFondationGrabber(.9);

        sleep(1000);

        //DriveFieldRealtiveDistance(.3, 270, 4); //drive away from stone
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(36)
                .build());

        sleep(100);

//        bot.followTrajectorySync(bot.trajectoryBuilder() //turn foundation
//        .splineTo(new Pose2d(50,-60,Math.toRadians(180)))
//                .build());
        for (int i = 0; i < 10; i++)
        {
            bot.turnRealtiveSync(180);
        }

        bot.setFondationGrabber(0);
        //DriveFieldRealtiveDistance(.5, 0, 1);

        sleep(500);


        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(24)
        .build());

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeLeft(20)
       .build());

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .forward(36)
       .build());

        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        telemetry.log().add("wrote file");

    }
}