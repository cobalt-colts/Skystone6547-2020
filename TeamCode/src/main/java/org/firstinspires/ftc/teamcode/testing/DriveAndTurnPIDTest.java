package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

@Autonomous
public class DriveAndTurnPIDTest extends SkyStone6547Meet2 {

    @Override
    public void runOpMode() {
        INIT(hardwareMap);

        initIMU();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.log().add("ready to start");
        waitForStart();

        //TurnPID(90,3);
        //TurnPID(0,3);

        DriveFieldRealtiveDistanceAndTurnPID(.5,0,3,90,1);
    }
}
