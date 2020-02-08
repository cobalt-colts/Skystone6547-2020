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
@Autonomous(name = "RED double skystone Road Runner", group = "auton")
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
        bot.setRunLift(true);

        //set the position of the bot
        bot.setPoseEstimate(new Pose2d(-36, -62,Math.toRadians(90)));

        telemetry.log().add("Ready to start");
        while (!isStarted())
        {
            bot.runAtAllTimes();
        }

        //Go to Skystones
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .strafeTo(new Vector2d(-36,-36))
                .build());
        //scan stones
        if (bot.isSkystone(bot.colorSensorSideLeft))
        {
            // ---SKYSTONE LEFT---
            bot.skyStoneLoc = SkyStoneLoc.LEFT;
            //back up a bit and strafe left
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .back(3)
                    .strafeLeft(15)
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
                    .back(3)
                    .strafeRight(15)
                    .build());
        }
        else
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .back(3)
            .build());
            //---SKYSTONE CENTER---
            //go back a bit
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

        }

        //open intake
        bot.outtake(1);
        sleep(500);
        bot.intake(1);

        //drive forward and intake stone
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(18)
        .build());

        //spline to under the skybridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
        .splineTo(new Pose2d(0,-41,Math.toRadians(180)))
                .build());

        bot.turnRealtiveSync(Math.toRadians(180));

        bot.setLiftTargetPos(bot.lift.getCurrentPosition() + 250);
        //bot.setLiftToTargetPos(bot.getLiftTargetPos(), 50);

        //drive forward a bit to relay the skystone in the build zone
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .back(14)
        .build());

        bot.moveLift(500,50);

        bot.setLiftTargetPos(bot.lift.getCurrentPosition() + 250);


        //go back under the skybridge
       bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(12)
        .build());

        bot.moveLift(0, 50);

        //spline to next to the other stones
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .splineTo(new Pose2d(-30,-40, Math.toRadians(180)))
        .build());

        bot.turnRealtiveSync(Math.toRadians(180));

        //prepare to grab the other Skystone
        if (bot.skyStoneLoc == SkyStoneLoc.RIGHT)
        {
            //if right, for forward a tiny bit and strafe
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .forward(4)
            .build());
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .strafeRight(25)
                    .build());
        }
        else if (bot.skyStoneLoc == SkyStoneLoc.CENTER)
        {
            //if center, go forward a bit and strafe
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .forward(13)
            .build());

            bot.followTrajectorySync(bot.trajectoryBuilder()
            .strafeRight(25)
            .build());


        }
        else if (bot.skyStoneLoc == SkyStoneLoc.LEFT)
        {
            //if left, go forward a lot and strafe
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(20)
                    .build());

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .strafeRight(25)
                    .build());
        }
        //grab SkyStone
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(8)
        .build());

        //get in path of skybridge


        bot.turnRealtiveSync(Math.toRadians(180));

        //go under skybridge and release stone
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(35)
                .build());

        bot.moveLift(500,50);

        //park under skybridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(9)
        .build());

        bot.moveLift(0,50);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(10)
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
