

package org.firstinspires.ftc.teamcode.oldPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet1.SkyStone6547;

/*
First auton for this year.
We tried having the robot move and use the color sensor at the same time to detect the skystone.
It failed becuase the robot would often miss the skystone

Obsolete: DO NOT USE
 */
@Autonomous(name = "skystone Autonomous test")
@Disabled
public class SkyStoneAutonmousTest extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();

        zeroEncoders();
        telemetry.log().add("ready to start");
        waitForStart();

        DriveFieldRealtiveDistance(1,90,2.9);

        if (isSkystone());
        {
            DriveFieldRealtiveDistance(.25,180,.5);
            if (!isSkystone())
            {
                DriveFieldRealtiveDistance(.25,180,.5);
            }
        }
        stopRobot();
        zeroEncoders();
        DriveFieldRealtiveDistance(.25, 180, .75);
        stopRobot();
        zeroEncoders();
        //intake(1);
        sleep(1);
        DriveFieldRealtiveDistance(.25,90,.25);
        DriveFieldRealtiveDistance(.7,270,2.5);
        TurnPID(270,3);
        DriveFieldRealtiveDistance(1, 0, 6);
    }
    boolean isSkystone()
    {
        return false;//colorSensorLeft.red() < 50;

    }




    // todo: write your code here
}

