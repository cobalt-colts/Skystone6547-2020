package org.firstinspires.ftc.teamcode.OldPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

/*
This auton is obsolete.  This auton is for an older verison of the robot and will not work with the current one
DO NOT USE
 */
@Disabled
public class newAuton extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();

        zeroEncoders();
        telemetry.log().add("ready to start");
        waitForStart();

        DriveFieldRealtiveDistance(1, 90, 3);

//        double distanceToDrive=6;
//        if (isSkystone(colorSensorLeft))
//        {
//            skyStoneLoc = SkyStoneLoc.LEFT;
//            DriveFieldRealtiveDistance(.25, 180, .3);
//            distanceToDrive-=.3;
//        }
//        else if (isSkystone(colorSensorRight))
//        {
//            skyStoneLoc = skyStoneLoc.RIGHT;
//            DriveFieldRealtiveDistance(.25, 0, .3);
//            distanceToDrive+=.3;
//        }
//        else skyStoneLoc = skyStoneLoc.CENTER;
        //intake(1);
//        DriveFieldRealtiveDistance(.3, 90, .5); //drive toward stone, intake it
//        DriveFieldRealtiveDistance(.3, 270, 2); //drive away from stone
//        DriveFieldRealtiveDistance(1, 0, distanceToDrive); //go under skybridge
//        DriveFieldRealtiveDistance(.5, 90, 2); //approuch foundation
        //outtake(1); //outtake stone to foundation
        sleep(1.0);

        while (opModeIsActive()); //don't test below this until the above works.  It grabs the second skystone

//        DriveFieldRealtiveDistance(.5, 270, 2);
//        DriveFieldRealtiveDistance(1, 180, distanceToDrive); //go under skybridge, to skystones
//        DriveFieldRealtiveDistance(.3, 90, .5); //drive toward stone, intake it
//        DriveFieldRealtiveDistance(.3, 270, 2); //drive away from stone
//        DriveFieldRealtiveDistance(1, 0, distanceToDrive); //go under skybridge
//        DriveFieldRealtiveDistance(.5, 90, 2); //approuch foundation
        //outtake(1); //outtake stone to foundation
        sleep(1.0);



    }
    boolean isSkystone(ColorSensor colorSensorToBeUsed)
    {
        return colorSensorToBeUsed.red() < 50;

    }
}
