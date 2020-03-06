package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@Autonomous(group = "drive")
public class strafeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .strafeLeft(24)
        .build());
    }
}
