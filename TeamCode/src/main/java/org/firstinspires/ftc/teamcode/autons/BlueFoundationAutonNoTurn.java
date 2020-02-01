package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;


/*
    This auton pulls the foundation to the build site and parks under the skybridge for the RED side
 */
@Autonomous(name = "BLUE Foundation Auton NO TURN road runner", group = "auton")
public class BlueFoundationAutonNoTurn extends LinearOpMode
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

        //wait for 10 seconds
        //sleep(10000);

        //set position of robot
        bot.setPoseEstimate(new Pose2d(35,62,Math.toRadians(90)));
        //robot starts backwards, so set the gyro Angle to 180 degrees
        bot.setAngleZzeroValue(180);
        telemetry.log().add("ready to start");

        waitForStart();

        //drive to foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
        .splineTo(new Pose2d(50,34,Math.toRadians(90)))
                .build());

        //grab foundation
        bot.setFondationGrabber(1);

        sleep(1000);

        //pull foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(41)
                .build());

        sleep(100);

        //pull foundation into wall
//        bot.followTrajectorySync(bot.trajectoryBuilder()
////                .back(24)
////                .build());

        //release foundation
        bot.setFondationGrabber(0);

        //strafe to skybrige
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .strafeLeft(65)
                .build());

        sleep(500);

        //save gyro angle

        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        while (opModeIsActive())
        {
            bot.moveLift(0,50);
            bot.outputTelemetry();
        }

    }
}
