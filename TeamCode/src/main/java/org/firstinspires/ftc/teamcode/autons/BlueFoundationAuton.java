package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;


/*
    This auton pulls the foundation to the build site and parks under the skybridge for the RED side
 */
@Autonomous(name = "BLUE Foundation Auton road runner", group = "auton")
public class BlueFoundationAuton extends LinearOpMode
{

    public void runOpMode()
    {
        DriveTrain6547State bot = new DriveTrain6547State(this); //the bot
          /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */
        bot.setLiftTargetPos(bot.liftStartingPos);
        bot.setRunLift(true);

        //set position of robot
        bot.setPoseEstimate(new Pose2d(35,62,Math.toRadians(90)));
        //robot starts backwards, so set the gyro Angle to 180 degrees
        bot.setAngleZzeroValue(180);
        telemetry.log().add("ready to start");

        waitForStart();

        //drive to foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
        .splineTo(new Pose2d(40,34,Math.toRadians(90)))
                .build());

        //grab foundation
        bot.setFondationGrabber(1);

        sleep(1000);

        //pull foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(36)
                .build());

        sleep(100);

        //turn 90 to put fondation to the wall
        for (int i = 0; i < 10; i++)
        {
            bot.turnRealtiveSync(Math.toRadians(0));
        }

        //release foundation grabber
        bot.setFondationGrabber(0);

        sleep(500);

        //drive foundation into wall
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(24)
                .build());

        //strafe to wall
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .strafeRight(20)
                .build());

        //drive forward to park under SkyBridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(36)
                .build());

        sleep(500);

        //save gyro angle

        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        while (opModeIsActive())
        {
            bot.outputTelemetry();
        }

    }
}
