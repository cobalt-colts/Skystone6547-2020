package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet1.SkyStone6547;

/*
This moves the linear slide down to level 0 (slightly above zero; the lowest position the linear slide can go)
 */
@Autonomous
@Disabled
public class ResetLift extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();
        zeroEncoders();
        telemetry.log().add("ready to start");

        waitForStart();

        setLiftLevel(0);

        sleep(1.0);
    }
}