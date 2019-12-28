package org.firstinspires.ftc.teamcode.oldPrograms.oldAutons.Meet2Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "BLUE skystone fondation meet 2")
@Disabled
public class BlueOneSkystoneFoundationSkystone extends SkyStone6547Meet2 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        angleZzeroValue=0;

        zeroEncoders();
        telemetry.log().add("ready to start");

        waitForStart();

        //setLiftLevel(0);

        DriveFieldRealtiveDistance(.3, 90, 2.7); //drive to stones
        DriveFieldRealtiveDistance(.15,90,.5);

        sleep(.25);
        TurnPID(0,1); //align to stones
        sleep(1.0);

        double distanceToDrive=8.2; //distance to drive to the other side
        if (isSkystone(colorSensorSideLeft)) //scan stones
        {
            skyStoneLoc = SkyStoneLoc.LEFT;
            DriveFieldRealtiveDistance(.15,270,.1);
            telemetry.log().add("LEFT");
            DriveFieldRealtiveDistance(.5, 180, .9);
            distanceToDrive-=.9;
        }
        else if (isSkystone(colorSensorSideRight))
        {
            telemetry.log().add("RIGHT");
            skyStoneLoc = SkyStoneLoc.RIGHT;
             DriveFieldRealtiveDistance(.15,270,.1);
            DriveFieldRealtiveDistance(.5, 0, .5);
            distanceToDrive+=.5;

        }
        else
        {
            telemetry.log().add("CENTER");
            skyStoneLoc = SkyStoneLoc.CENTER;
             DriveFieldRealtiveDistance(.15,270,.1);
            // DriveFieldRealtiveDistance(.5, 180, .8);
            distanceToDrive+=0;

        }
        telemetry.log().add("RIGHT SKYSTONE: " + colorSensorSideRight.red());
        telemetry.log().add("LEFT SKYSTONE: " + colorSensorSideLeft.red());
        
        intake(1);

        DriveFieldRealtiveDistance(.3, 90, 2); //drive toward stone
        //DriveFieldRealtiveDistance(.25,0,.9); //grab stone

        sleep(.2);

        DriveFieldRealtiveDistance(.5, 270, 2.2); //drive away from stone
        //grabBlock();
        sleep(.2);
        TurnPID(270,3.5);
        //setGrabber(1);
        DriveFieldRealtiveDistance(.5,180,distanceToDrive); //go to foundation
        //grabBlock();
        sleep(.2);
        TurnPID(180,2.75); //align to stones
        
        DriveFieldRealtiveDistance(.3,90,.7); //approuch foundation
        
        setFondationGrabber(1);
        sleep(1.0);
        DriveFieldRealtiveDistance(3,270,.2);
        grabBlock();
        sleep(.2);
        setLiftLevel(1); //drop stone
        releaseGrabbers();
        sleep(.25);
        grabBlock();
        
        setLiftLevel(0);
        
        DriveFieldRealtiveDistance(.5,270,2); //pull foundation
        sleep(.2);
        
        TurnPID(270,2.75,1.5); //turn foundation
        sleep(.2);
        setFondationGrabber(0);
        //DriveFieldRealtiveDistance(.3,270,.35); //go sideways
        DriveFieldRealtiveDistance(.5,180,3); //push fondation
        
        //telemetry.log().add("Done");
        
        DriveFieldRealtiveDistance(.3,90,.6); //move sideways

        DriveFieldRealtiveDistance(.4, 0,4.5); //go under skybridge

        releaseGrabbers();

        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("wrote file");



    }
}
