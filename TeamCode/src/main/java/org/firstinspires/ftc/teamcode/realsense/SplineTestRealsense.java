package org.firstinspires.ftc.teamcode.realsense;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveTrain6547Offseason;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Config
public class SplineTestRealsense extends LinearOpMode {

    public static double X = 30;
    public static double Y = 30;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Realsense drive = new DriveTrain6547Realsense(this);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Vector2d(X, Y), 0)
                        .build()
        );

        sleep(500);

        drive.turnRealtiveSync(Math.toRadians(225));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Vector2d(0,0),Math.toRadians(180))
                        .build()
        );

        sleep(500);
        drive.turnSync(Math.toRadians(180));
    }
}
