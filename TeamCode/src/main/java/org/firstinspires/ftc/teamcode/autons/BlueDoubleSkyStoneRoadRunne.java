package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class BlueDoubleSkyStoneRoadRunne extends LinearOpMode {

    public void runOpMode()
    {
        DriveTrain6547 bot = new DriveTrain6547(this);

        bot.setAngleZzeroValue(0);

        bot.setPoseEstimate(new Pose2d(-36, 60,Math.toRadians(-90)));

        telemetry.log().add("Ready to start");
        waitForStart();

        if (isStopRequested()) return;

        bot.driveForward(23);

        bot.outputTelemetry();

        if (bot.isSkystone(bot.colorSensorSideLeft)) //scan stones
        {
            bot.skyStoneLoc = SkyStoneLoc.LEFT;
            bot.driveBackward(2);

            telemetry.log().add("LEFT");

            bot.strafeLeft(5);

        }
        else if (bot.isSkystone(bot.colorSensorSideRight))
        {
            telemetry.log().add("RIGHT");
            bot.skyStoneLoc = SkyStoneLoc.RIGHT;
            bot.driveBackward(2);
            bot.strafeRight(5);

        }
        else
        {
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

            bot.driveBackward(2);

        }
        bot.intake(1);

        bot.driveForward(12);
        //sleep(250);

        bot.driveBackward(14);

        bot.outputTelemetry();
        //sleep(.25);
        //bot.turnRealtiveSync(90);

        //bot.strafeLeft(60);

        bot.DriveFieldRealtiveDistance(.5,0,5);

        bot.outputTelemetry();

        bot.strafeRight(100);

        bot.turnRealtiveSync(0);
        //setGrabber(1);
        while (opModeIsActive());

        telemetry.log().add("wrote file");

    }
}
