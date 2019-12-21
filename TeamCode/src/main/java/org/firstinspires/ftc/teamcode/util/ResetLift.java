package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

/*
This moves the linear slide down to level 0 (slightly above zero; the lowest position the linear slide can go)
 */
@Autonomous
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