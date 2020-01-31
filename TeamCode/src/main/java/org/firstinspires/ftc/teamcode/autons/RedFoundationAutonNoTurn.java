package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;


/*
    This auton pulls the foundation to the build site and parks under the skybridge for the RED side
 */
@Autonomous(name = "RED Foundation Auton NO TURN road runner", group = "auton")
public class RedFoundationAutonNoTurn extends LinearOpMode
{

    public void runOpMode()
    {
        DriveTrain6547 bot = new DriveTrain6547(this); //the bot
          /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */
        bot.setLiftTargetPos(bot.liftStartingPos);
        bot.setRunLift(true);

        //set position of robot
        bot.setPoseEstimate(new Pose2d(35,-62,Math.toRadians(270)));
        //robot starts is backwards, so set the gyro Angle to 180 degrees
        bot.setAngleZzeroValue(180);

        //wait for 10 seconds
        //sleep(10000);

        telemetry.log().add("ready to start");

        waitForStart();

        //drive to foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
        .splineTo(new Pose2d(50,-34,Math.toRadians(270)))
                .build());

        //grab foundation
        bot.setFondationGrabber(1);

        sleep(1000);

        //pull foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(43)
                .build());

        bot.setFondationGrabber(0);

        sleep(100);

        //park to foundation
       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeRight(65)
       .build());

       //save gyro angle
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        while (opModeIsActive())
        {
            bot.outputTelemetry();
        }

    }
}
