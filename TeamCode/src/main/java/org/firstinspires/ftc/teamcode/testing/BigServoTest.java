package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

@TeleOp
@Disabled
public class BigServoTest extends SkyStone6547 {


    public void runOpMode()
    {
        INIT(hardwareMap);
        setSpinner(0);
        telemetry.log().add("ready to start tt");
        waitForStart();

        setSpinner(1);

        sleep(5.0);
    }
}