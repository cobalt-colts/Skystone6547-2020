package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@Autonomous(name = "Drive Forward Test",group = "test")
public class DriveForwardTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("ready to start");
        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(24)
        .build());
    }
}
