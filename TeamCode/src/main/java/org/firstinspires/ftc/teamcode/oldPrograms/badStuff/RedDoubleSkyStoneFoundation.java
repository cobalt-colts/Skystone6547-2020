package org.firstinspires.ftc.teamcode.oldPrograms.badStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.actions.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.IntakeUntilStone;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED double skystone Foundation State", group = "auton")
@Disabled
public class RedDoubleSkyStoneFoundation extends LinearOpMode {

    public void runOpMode()
    {
        //make bot fast
        DriveConstants.BASE_CONSTRAINTS =  new DriveConstraints(
                80, 40, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0);

        DriveTrain6547State bot = new DriveTrain6547State(this); //the robot

        /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */

        bot.setLiftTargetPos(bot.liftStartingPos);
        bot.setRunLift(false);

        bot.openGrabber();

        //set the position of the bot
        bot.setPoseEstimate(new Pose2d(-36, -62,Math.toRadians(90)));

        telemetry.log().add("Ready to start");
        while (!isStarted())
        {
            bot.runAtAllTimes();
        }

        //Go to Skystones
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .splineTo(new Pose2d(-35,-31,Math.toRadians(90)))
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
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//            .back(5)
//            .build());
//            //open intake
//            bot.setRunIntakeUntilStone(true);
//
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//            .splineTo(new Pose2d(-46,-24,Math.toRadians(135)))
//            .build());

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(4)
                    .addTemporalMarker(0,new IntakeUntilStone(bot))
            .strafeTo(new Vector2d(-52,-30))
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
                    .back(4)
                    .addTemporalMarker(0,new IntakeUntilStone(bot))
                    .strafeTo(new Vector2d(-18,-30))
                    .build());
        }
        else
        {
            //open intake
            bot.setRunIntakeUntilStone(true);
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(-36,-20,Math.toRadians(90)))
            .build());
            //---SKYSTONE CENTER---
            //go back a bit
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

        }

        if (bot.getRunIntakeUntilStone())
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .forward(12)
            .build());
        }

        //open intake

        //drive forward and intake stone
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//                .forward(18)
//        .build());

        //spline to under the skybridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
                // .reverse
                .splineTo(new Pose2d(0,-40,Math.toRadians(180)))
                .splineTo(new Pose2d(35,-40,Math.toRadians(180)))
                .splineTo(new Pose2d(45,-21,Math.toRadians(270)))
                .build());

        bot.setFondationGrabber(1);

        sleep(500);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(20)
                .splineTo(new Pose2d(33, -60, Math.toRadians(180)))
                .build());

        bot.turnSync(Math.toRadians(-200));

        //go to other stone

        bot.setFondationGrabber(0);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(24)
                .addTemporalMarker(1,new Intake(bot,1))
        .splineTo(new Pose2d(0,-40,Math.toRadians(180)))
        .build());

        bot.setRunIntakeUntilStone(true);

        double constX= -25;
        double constY = -34;

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
                    .splineTo(new Pose2d(constX,constY,Math.toRadians(180)))
            .splineTo(new Pose2d(-45,-22,Math.toRadians(135)))
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
                    .splineTo(new Pose2d(constX-5,constY,Math.toRadians(180)))
            .splineTo(new Pose2d(-50,-22, Math.toRadians(130)))
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
                    .splineTo(new Pose2d(constX,constY,Math.toRadians(180)))
                    .splineTo(new Pose2d(-60,-22,Math.toRadians(140)))
                    .build());
        }

        if (bot.getRunIntakeUntilStone()) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(8)
                    .build());
        }


        //go under skybridge and release stone
       bot.followTrajectorySync(bot.trajectoryBuilder()
               // .reverse
       .splineTo(new Pose2d(0,-36,Math.toRadians(180)))
               .addTemporalMarker(1.5,new Intake(bot,1))
               .splineTo(new Pose2d(-8,-36,Math.toRadians(180)))
       .build());

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeTo(new Vector2d(0,-33))
       .build());

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeRight(10)
       .build());

       bot.stopIntake();


        //save gyro angle
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        telemetry.log().add("wrote file at " + bot.getIMUAngle() + " degrees");
        //run until auton is over to keep the scissor lift down
        while (opModeIsActive()) {
            bot.outputTelemetry();
        }

    }
}
