package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;
import org.opencv.core.Mat;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED double skystone State", group = "auton")
public class RedDoubleSkyStoneRoadRunne extends LinearOpMode {

    public void runOpMode()
    {
        DriveTrain6547 bot = new DriveTrain6547(this); //the robot

        /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */
        bot.setLiftTargetPos(bot.liftStartingPos);
        bot.setRunLift(false);

        //set the position of the bot
        bot.setPoseEstimate(new Pose2d(-36, -62,Math.toRadians(90)));

        telemetry.log().add("Ready to start");
        while (!isStarted())
        {
            bot.runAtAllTimes();
        }

        //Go to Skystones
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .splineTo(new Pose2d(-35,-32,Math.toRadians(90)))
        .build());

        sleep(500);
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//                .strafeTo(new Vector2d(-36,-36))
//                .build());
        //scan stones
        if (bot.isSkystone(bot.colorSensorSideLeft))
        {
            // ---SKYSTONE LEFT---
            bot.skyStoneLoc = SkyStoneLoc.LEFT;
            //back up a bit and strafe left
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .back(5)
            .build());
            //open intake
            bot.intake(1);

            bot.followTrajectorySync(bot.trajectoryBuilder()
            .splineTo(new Pose2d(-46,-24,Math.toRadians(135)))
            .build());
            telemetry.log().add("LEFT");

        }
        else if (bot.isSkystone(bot.colorSensorSideRight))
        {
            //---SKYSTONE RIGHT---
            telemetry.log().add("RIGHT");
            bot.skyStoneLoc = SkyStoneLoc.RIGHT;
            //drive back a bit and strafe right
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(5)
                    .build());
            //open intake
            bot.intake(1);

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(-26,-24,Math.toRadians(135)))
                    .build());
        }
        else
        {
            //open intake
            bot.intake(1);
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(-36,-24,Math.toRadians(90)))
            .build());
            //---SKYSTONE CENTER---
            //go back a bit
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

        }

        //open intake
        bot.intake(1);

        //drive forward and intake stone
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//                .forward(18)
//        .build());

        //spline to under the skybridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
        .splineTo(new Pose2d(0,-38,Math.toRadians(180)))
                .splineTo(new Pose2d(15,-38,Math.toRadians(180)))
                .build());

        bot.turnRealtiveSync(Math.toRadians(180));


        //go to other stone
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .splineTo(new Pose2d(-25,-30,Math.toRadians(180)))
        .build());

        bot.turnRealtiveSync(Math.toRadians(180));

        //prepare to grab the other Skystone
        if (bot.skyStoneLoc == SkyStoneLoc.RIGHT)
        {
            //if right, for forward a tiny bit and strafe
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//            .forward(4)
//            .build());
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//            .strafeRight(25)
//                    .build());
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .splineTo(new Pose2d(-40,-20,Math.toRadians(135)))
            .build());
        }
        else if (bot.skyStoneLoc == SkyStoneLoc.CENTER)
        {
            //if center, go forward a bit and strafe
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//            .forward(13)
//            .build());
//
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//            .strafeRight(15)
//            .build());
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .splineTo(new Pose2d(-50,-22, Math.toRadians(135)))
            .build());


        }
        else if (bot.skyStoneLoc == SkyStoneLoc.LEFT)
        {
            //if left, go forward a lot and strafe
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//                    .forward(20)
//                    .build());
//
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//                    .strafeRight(15)
//                    .build());

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(-58,-20,Math.toRadians(135)))
                    .build());
        }


        //go under skybridge and release stone
       bot.followTrajectorySync(bot.trajectoryBuilder()
               .reverse()
       .splineTo(new Pose2d(0,-36,Math.toRadians(180)))
               .splineTo(new Pose2d(-10,-36,Math.toRadians(180)))
       .build());

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeTo(new Vector2d(0,-36))
       .build());


        //save gyro angle
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        telemetry.log().add("wrote file at " + bot.getIMUAngle() + " degrees");
        //run until auton is over to keep the scissor lift down
        while (opModeIsActive()) {
            bot.outputTelemetry();
        }

    }
}
