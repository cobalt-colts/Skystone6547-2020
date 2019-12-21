package org.firstinspires.ftc.teamcode.OldPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;

/*
This auton goal is to grabs two skystones and deploy both to the fondation and then parks under the skybridge
 */
@Autonomous(name = "RED skystone to skystone")
@Disabled
public class NewAuton_Copy2 extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();
        angleZzeroValue=-90;

        zeroEncoders();
        telemetry.log().add("ready to start");
        waitForStart();

        DriveFieldRealtiveDistance(.5, 90, 2.9);
        DriveFieldRealtiveDistance(.25,90,.25);

        sleep(.5);
        double distanceToDrive=2;
        if (isSkystone(colorSensorSideLeft))
        {
            skyStoneLoc = SkyStoneLoc.LEFT;
            DriveFieldRealtiveDistance(.5, 0, .4);
            distanceToDrive-=.4;
        }
        else if (isSkystone(colorSensorSideRight))
        {
            skyStoneLoc = SkyStoneLoc.RIGHT;
            DriveFieldRealtiveDistance(.25, Math.toDegrees(Math.atan(4/20)), Math.hypot(4,20));
            //driveCircle(.25, 1.4, 60, true, true);
            DriveFieldRealtiveDistance(.25, 35, 1);
            //TurnPID(20,1);
            //driveCircle(.25, 1.6, 45, false, true);
            distanceToDrive+=0;
        }
        else
        {
            skyStoneLoc = SkyStoneLoc.CENTER;
            DriveFieldRealtiveDistance(.5, 0, .8);
            distanceToDrive-=.8;

        }
        if (skyStoneLoc != SkyStoneLoc.RIGHT)
        {
            DriveFieldRealtiveDistance(.25, 90, 1.3); //drive toward stone
            DriveFieldRealtiveDistance(.25,180,.9); //grab stone
        }
        sleep(.5);
        DriveFieldRealtiveDistance(.5, 270, 1.3); //drive away from stone
        DriveFieldRealtiveDistance(.5,180, .2);
        //DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(distanceToDrive/2))+270, Math.hypot(1, distanceToDrive/2),false);
        //DriveFieldRealtiveDistance(1, 0, distanceToDrive/4); //go under skybridge
        //DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(1/(distanceToDrive/2))), Math.hypot(1, distanceToDrive/2));
        setGrabber(1);
        sleep(.5);
        DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(2.083/distanceToDrive))-90,Math.hypot(25/12,distanceToDrive));
        DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(1.5/5.2)),Math.hypot(1.5,5.2));

        //driveCircle(.25, 1.6, 90, true, false);
        //TurnPID(180, 2);
        DriveFieldRealtiveDistance(.5, 90, .5);
        //  outtake(1); //outtake stone to foundation
        sleep(1.0);

        //while (opModeIsActive()); //don't test below this until the above works.  It grabs the second skystone

        distanceToDrive+=2.4;
        DriveFieldRealtiveDistance(.5, 270, .5); //backup from foundation
        TurnPID(90,2);
        DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan(1.5/6))+180, Math.hypot(1.5, 6),false);
        //DriveFieldRealtiveDistance(1, 180, distanceToDrive); //go under skybridge, to skystones
        //DriveFieldRealtiveDistance(1,Math.toDegrees(Math.atan((distanceToDrive/2)/2))+90, Math.hypot(2, distanceToDrive/2));
        //DriveFieldRealtiveDistance(.5,Math.toDegrees(Math.atan((distanceToDrive/2)/1.5))+90, Math.hypot(1.5,distanceToDrive/2));
        DriveFieldRealtiveDistance(.5, Math.toDegrees(Math.atan(distanceToDrive))+90, Math.hypot(distanceToDrive,1)); //under skybrige, to skystone
        DriveFieldRealtiveDistance(.25, 90, 2); //drive toward stone, intake it
        DriveFieldRealtiveDistance(.25, 180, 1);
        DriveFieldRealtiveDistance(.5, 270, 1.5); //drive away from stone
        DriveFieldRealtiveDistance(.5, Math.toDegrees(Math.atan((distanceToDrive)/1.5))+270, Math.hypot(1.5,distanceToDrive), false);
        DriveFieldRealtiveDistance(.5, Math.toDegrees(Math.atan(1.5/5)), Math.hypot(1.5,5)); //go under skybridge
        DriveFieldRealtiveDistance(.25, 90, .5); //approuch foundation
        TurnPID(180,2);
        // outtake(1); //outtake stone to foundation
        sleep(1.0);
        DriveFieldRealtiveDistance(.5, 270, .5);
        DriveFieldRealtiveDistance(.5, Math.toDegrees(Math.atan(1/5))+180,Math.hypot(1, 5));
        sleep(1.0);



    }
    boolean isSkystone(ColorSensor colorSensorToBeUsed)
    {
        return colorSensorToBeUsed.red() < 50;

    }
}
