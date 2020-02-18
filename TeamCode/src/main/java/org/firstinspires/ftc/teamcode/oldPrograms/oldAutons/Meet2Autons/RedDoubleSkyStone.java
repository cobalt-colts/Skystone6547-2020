package org.firstinspires.ftc.teamcode.oldPrograms.oldAutons.Meet2Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED double skystone meet 2")
@Disabled
public class RedDoubleSkyStone extends SkyStone6547Meet2 {

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
        double distanceToDrive = 5;
        if (isSkystone(colorSensorSideLeft)) //scan stones
        {
            skyStoneLoc = SkyStoneLoc.LEFT;
            DriveFieldRealtiveDistance(.15,270,.1);
            telemetry.log().add("LEFT");
            DriveFieldRealtiveDistance(.5, 180, .7);
            distanceToDrive-=.7;
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
        intake(1);

        DriveFieldRealtiveDistance(.3, 90, 2); //drive toward stone
        //DriveFieldRealtiveDistance(.25,0,.9); //grab stone

        sleep(.25);

        DriveFieldRealtiveDistance(.5, 270, 2.1); //drive away from stone
        sleep(.25);
        TurnPID(90,4);
        //setGrabber(1);
        DriveFieldRealtiveDistance(.5,0,distanceToDrive); //go to other slide
        releaseGrabbers();
        //TurnPID(0,1); //turn around
        //TurnPID(270,2);

        if (skyStoneLoc!=SkyStoneLoc.LEFT) {
            DriveFieldRealtiveDistance(.5, 180, 8); //go under skybridge, to other
        }
        else
        {
            DriveFieldRealtiveDistance(.3, 180, 2); //done with auton
            while (opModeIsActive()) outputTelemetry();
        }

        TurnPID(0,3); //turn to face stones
        sleep(.25);
        intake(1);
        DriveFieldRealtiveDistance(.5, 180, 1);
        //DriveFieldRealtiveDistance(.4,180,.2);
        if (skyStoneLoc == SkyStoneLoc.CENTER)
        {
            DriveFieldRealtiveDistance(.4, 0, .5);
        }
        else if (skyStoneLoc == SkyStoneLoc.RIGHT)
        {
            DriveFieldRealtiveDistance(.3, 0, 1.1);
        }
        else if (skyStoneLoc == SkyStoneLoc.LEFT)
        {
            telemetry.log().add("DONE");
            while (opModeIsActive()) //wait until done
            {
                outputTelemetry();
            }
        }
        
        DriveFieldRealtiveDistance(.5, 90, 2.2); //drive toward stone
        //DriveFieldRealtiveDistance(.25,0,.9); //grab stone

        sleep(.25);

        DriveFieldRealtiveDistance(.5, 270, 2.3); //drive away from stone
        
        sleep(.25);
        
        TurnPID(90,3);

        DriveFieldRealtiveDistance(.5, 0,8);
        
        sleep(.25);
        
        DriveFieldRealtiveDistance(.5,180,1.5);
        
        sleep(.25);
        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("wrote file");

    }
}
