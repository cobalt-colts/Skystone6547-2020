package org.firstinspires.ftc.teamcode.realsense;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Realsense Auto")
public class RoadRunnerAutonRealsenseTest extends LinearOpMode {

    DriveTrain6547Realsense bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new DriveTrain6547Realsense(this);

        waitForStart();

        bot.startRealsense();

        bot.followTrajectorySync(bot.trajectoryBuilder().splineTo(new Vector2d(20,20),0).build());


    }
}
