package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.actions.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@TeleOp(name = "Smaat Paak RED")
@Disabled
public class SmaatPaak extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain6547State bot = new DriveTrain6547State(this);

        bot.setPoseEstimate(bot.readRobotPos());

        telemetry.addData("Ready to start","");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            bot.followTrajectory(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(-10,-20,Math.toRadians(30)))
            .splineTo(new Pose2d(-60,50,Math.toRadians(90)))
            .build());

            bot.setRunIntakeUntilStone(true);

            //go back and fourth until the robot gets a stone
            while (bot.getRunIntakeUntilStone()) {
                if (bot.getRunIntakeUntilStone()) {
                    bot.followTrajectory(bot.trajectoryBuilder()
                            .forward(8)
                            .build());
                }
                if (bot.getRunIntakeUntilStone())
                {
                    bot.followTrajectory(bot.trajectoryBuilder()
                            .back(8)
                            .build());
                }
            }

            bot.followTrajectory(bot.trajectoryBuilder()
                    //.reverse()
            .splineTo(new Pose2d(-20,-20,Math.toRadians(135)))
            .splineTo(new Pose2d(0,-40,Math.toRadians(180)))
                    //.addMarker(new Intake(bot,1))
                    .splineTo(new Pose2d(0,10))
            .build());


        }

    }
}
