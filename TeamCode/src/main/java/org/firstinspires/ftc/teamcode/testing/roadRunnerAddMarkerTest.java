package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.actions.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@Autonomous
@Disabled
public class roadRunnerAddMarkerTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.addData("Ready to start","");
        telemetry.update();

        waitForStart();

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(24)
        //.addMarker(new Intake(bot,1))
        .forward(24)
        .build());
    }
}
