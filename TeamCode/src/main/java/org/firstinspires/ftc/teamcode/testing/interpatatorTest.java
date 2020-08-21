package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

@Autonomous(group = "test")
public class interpatatorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("Ready to start");

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .splineTo(new Pose2d(20,20,Math.toRadians(0)))
                .build());

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .splineTo(new Pose2d(50,0,Math.toRadians(0)))
                .build());


    }
}
