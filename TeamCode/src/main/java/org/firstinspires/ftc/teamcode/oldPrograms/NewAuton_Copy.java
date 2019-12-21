package org.firstinspires.ftc.teamcode.OldPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

/**
 *This auton is obsolete.  This auton is for an older verison of the robot and will not work with the current one
 *
 * Main deference: This uses tangents and Math.hypot to move diagonally.
 */
@Autonomous(name = "newAuton with diagnals RED")
@Disabled
public class NewAuton_Copy extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();

        zeroEncoders();
        telemetry.log().add("ready to start");
        waitForStart();

        DriveFieldRealtiveDistance(.5, 90, 2.8);
        DriveFieldRealtiveDistance(.25,90,.25);

        sleep(.5);
        double distanceToDrive=10;
//        if (isSkystone(colorSensorLeft))
//        {
//            skyStoneLoc = SkyStoneLoc.LEFT;
//            DriveFieldRealtiveDistance(.5, 180, .9);
//            distanceToDrive+=.9;
//        }
//        else if (isSkystone(colorSensorRight))
//        {
//            skyStoneLoc = skyStoneLoc.RIGHT;
//            DriveFieldRealtiveDistance(.5, 0, .9);
//            distanceToDrive-=.9;
//        }
//        else skyStoneLoc = skyStoneLoc.CENTER;
        //intake(1);
        DriveFieldRealtiveDistance(.25, 90, 1); //drive toward stone, intake it
        sleep(.5);
        DriveFieldRealtiveDistance(.5, 270, 1); //drive away from stone
        DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(distanceToDrive/2))+270, Math.hypot(1, distanceToDrive/2),false);
        //DriveFieldRealtiveDistance(1, 0, distanceToDrive/4); //go under skybridge
        DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(1/(distanceToDrive/2))), Math.hypot(1, distanceToDrive/2));
        TurnPID(180, 3);
        DriveFieldRealtiveDistance(.25, 90, .5); //approuch foundation
        //  outtake(1); //outtake stone to foundation
        sleep(1.0);

        //while (opModeIsActive()); //don't test below this until the above works.  It grabs the second skystone

        distanceToDrive+=2.9;
        DriveFieldRealtiveDistance(.5, 270, .3); //backup from foundation
        TurnPID(0, 3);
        DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(1.5/(distanceToDrive/2)))+180, Math.hypot(1.5, distanceToDrive/2),false);
        //DriveFieldRealtiveDistance(1, 180, distanceToDrive); //go under skybridge, to skystones
        //DriveFieldRealtiveDistance(1,Math.toDegrees(Math.atan((distanceToDrive/2)/2))+90, Math.hypot(2, distanceToDrive/2));
        //DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan((distanceToDrive/2)/1.5))+90, Math.hypot(1.5,distanceToDrive/2));
        DriveFieldRealtiveDistance(.5, 180, distanceToDrive/2); //under skybrige, to skystone
        DriveFieldRealtiveDistance(.25, 90, 1); //drive toward stone, intake it
        DriveFieldRealtiveDistance(.5, 270, .5); //drive away from stone
        DriveFieldRealtiveDistance(.5, Math.toDegrees(Math.atan((distanceToDrive/2)/1.5))+270, Math.hypot(1.5,distanceToDrive/2), false);
        DriveFieldRealtiveDistance(.5, Math.toDegrees(Math.atan(1.5/(distanceToDrive/2))),  Math.hypot(1.5,distanceToDrive/2)); //go under skybridge
        DriveFieldRealtiveDistance(.25, 90, .5); //approuch foundation
        // outtake(1); //outtake stone to foundation
        sleep(1.0);
        DriveFieldRealtiveDistance(.5, 270, .5);
        DriveFieldRealtiveDistance(.5, Math.toDegrees(Math.atan(1/5))+180,Math.hypot(1, 5));



    }
    boolean isSkystone(ColorSensor colorSensorToBeUsed)
    {
        return colorSensorToBeUsed.red() < 50;

    }
}
