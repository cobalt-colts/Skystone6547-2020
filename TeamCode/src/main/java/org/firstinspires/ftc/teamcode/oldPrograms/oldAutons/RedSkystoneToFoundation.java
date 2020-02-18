package org.firstinspires.ftc.teamcode.OldPrograms.oldAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED skystone to Fondation")
@Disabled
public class RedSkystoneToFoundation extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();
        angleZzeroValue=-90;

        zeroEncoders();
        telemetry.log().add("ready to start");

        waitForStart();

        //setLiftLevel(0);

        setGrabber(0);
        sleep(1.0);

        DriveFieldRealtiveDistance(.5, 90, 2.9);
        DriveFieldRealtiveDistance(.25,90,.25);
        sleep(.5);
        TurnPID(270,1);
        sleep(.5);
        double distanceToDrive=7.75;
        if (isSkystone(colorSensorSideLeft))
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
            DriveFieldRealtiveDistance(.5, 180, .8);
            distanceToDrive+=.8;

        }
        telemetry.log().add("RIGHT SKYSTONE: " + colorSensorSideRight.red());
        telemetry.log().add("LEFT SKYSTONE: " + colorSensorSideLeft.red());

        DriveFieldRealtiveDistance(.25, 90, 1.3); //drive toward stone
        DriveFieldRealtiveDistance(.25,0,.9); //grab stone

        sleep(.5);

        DriveFieldRealtiveDistance(.5, 270, 2.2); //drive away from stone
        setSpinner(.94);
        sleep(.5);
        setGrabber(1);
        sleep(.5);
        DriveFieldRealtiveDistance(.5,0,distanceToDrive); //go to wall, close to fondation
        TurnPID(180, 3);
        setLiftLevel(1);
        DriveFieldRealtiveDistance(.25, 90, 1.5); //approuch fonation
        setFondationGrabber(.6);
        setSpinner(0);
        sleep(.5);
        setGrabber(0);
        sleep(.5);
        setSpinner(1);
        setGrabber(1);
        DriveFieldRealtiveDistance(.3, 270, 9); //drive away from fondation
        setFondationGrabber(0);
        sleep(.5);
        setLiftLevel(0);
        DriveFieldRealtiveDistance(.5, 180, 3.7); //park under skybridge
        sleep(.5);
        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("wrote file");

    }
    boolean isSkystone(ColorSensor colorSensorToBeUsed)
    {
        return colorSensorToBeUsed.red() < 50;

    }
}
