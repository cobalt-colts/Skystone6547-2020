package org.firstinspires.ftc.teamcode.oldPrograms.oldAutons.Meet2Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED two skystone inside meet 2")
@Disabled
public class RedOneSkystoneInsideMeet2 extends SkyStone6547Meet2 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        angleZzeroValue=0;

        zeroEncoders();
        telemetry.log().add("ready to start");

        waitForStart();

        //setLiftLevel(0);

        DriveFieldRealtiveDistance(.4, 90, 2.5); //drive to stones
        DriveFieldRealtiveDistance(.25,90,.25);

        sleep(.5);
        TurnPID(0,1); //align to stones
        sleep(.5);

        double distanceToDrive=5; //distance to drive to the other side
        if (isSkystone(colorSensorSideLeft)) //scan stones
        {
            skyStoneLoc = SkyStoneLoc.LEFT;
            telemetry.log().add("LEFT");
            DriveFieldRealtiveDistance(.5, 180, 1.5);
            distanceToDrive+=1.5;
        }
        else if (isSkystone(colorSensorSideRight))
        {
            telemetry.log().add("RIGHT");
            skyStoneLoc = SkyStoneLoc.RIGHT;
            DriveFieldRealtiveDistance(.5, 180, .2);
            distanceToDrive+=.2;

        }
        else
        {
            telemetry.log().add("CENTER");
            skyStoneLoc = SkyStoneLoc.CENTER;
            // DriveFieldRealtiveDistance(.5, 180, .8);
            distanceToDrive+=.8;

        }
        telemetry.log().add("RIGHT SKYSTONE: " + colorSensorSideRight.red());
        telemetry.log().add("LEFT SKYSTONE: " + colorSensorSideLeft.red());
        
        intake(1);

        DriveFieldRealtiveDistance(.25, 90, 2); //drive toward stone
        //DriveFieldRealtiveDistance(.25,0,.9); //grab stone

        sleep(.5);

        DriveFieldRealtiveDistance(.5, 270, 2.65); //drive away from stone
        sleep(.25);
        TurnPID(90,2); //align to stones
        //setGrabber(1);
        sleep(.25);
        DriveFieldRealtiveDistance(.5,0,distanceToDrive); //go to wall, close to fondation

        //DriveFieldRealtiveDistance(.5, 90, 2); //go to fondation
        //  outtake(1); //outtake stone to foundation
        //setLiftLevel(1);
        //setSpinner(1);
        //setGrabber(0);
        sleep(.25);
        TurnPID(90,.5); //align to stones

        //while (opModeIsActive()); //don't test below this until the above works.  It grabs the second skystone

        //setFondationGrabber(1);

        //DriveFieldRealtiveDistance(.5, 270,1); //go under skybridge
        //TurnPID(180,.5); //align to stones
        DriveFieldRealtiveDistance(.5, 180,8); //go under skybridge
        sleep(.25);
        TurnPID(0,3);
        DriveFieldRealtiveDistance(.4,180,1);
        if (skyStoneLoc == SkyStoneLoc.CENTER)
        {
            DriveFieldRealtiveDistance(.4, 0, .6);
        }
        else if (skyStoneLoc == SkyStoneLoc.RIGHT)
        {
            DriveFieldRealtiveDistance(.4, 0, 1.1);
        }
        else if (skyStoneLoc == SkyStoneLoc.LEFT)
        {
        
        }
        
        DriveFieldRealtiveDistance(.25, 90, 3.2); //drive toward stone
        //DriveFieldRealtiveDistance(.25,0,.9); //grab stone

        sleep(.25);

        DriveFieldRealtiveDistance(.5, 270, 2.1); //drive away from stone
        
        sleep(.25);
        
        TurnPID(90,2.5);
        
        sleep(.25);
        
        DriveFieldRealtiveDistance(.5, 0,8);
        
        TurnPID(90,.5);
        
        DriveFieldRealtiveDistance(.5, 180,1.5);
        
        sleep(.5);
        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("wrote file");

    }
}
