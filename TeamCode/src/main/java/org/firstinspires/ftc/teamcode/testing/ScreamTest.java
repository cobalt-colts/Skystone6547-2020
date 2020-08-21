package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

@TeleOp
@Disabled
public class ScreamTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        bot.scream();
        telemetry.log().add("Scream");

        sleep(2000);
    }
}
