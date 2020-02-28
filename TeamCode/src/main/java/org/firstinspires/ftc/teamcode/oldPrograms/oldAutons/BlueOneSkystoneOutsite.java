package org.firstinspires.ftc.teamcode.OldPrograms.oldAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet1.SkyStone6547;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;

/*
This auton moves one skystone to the other half of the field and then parks under the skybridge for the BLUE side.
Note: main difference between red and blue side is the robot moves in opposite directions to move to other side of the field
 */
@Autonomous(name = "BLUE one skystone ouside")
@Disabled
public class BlueOneSkystoneOutsite extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();
        angleZzeroValue=-90;

        zeroEncoders();

        telemetry.log().add("ready to start");

        waitForStart();

        setGrabber(0);
        sleep(1.0);

        DriveFieldRealtiveDistance(.5, 90, 2.9);
        DriveFieldRealtiveDistance(.25,90,.4);
        sleep(.5);
        TurnPID(270,1);
        sleep(.5);
        double distanceToDrive=6;
        if (isSkystone(colorSensorSideLeft))
        {
            skyStoneLoc = SkyStoneLoc.LEFT;
            telemetry.log().add("LEFT");
            DriveFieldRealtiveDistance(.5, 0, .2);
            distanceToDrive+=0.1;
        }
        else if (isSkystone(colorSensorSideRight))
        {
            telemetry.log().add("RIGHT");
            skyStoneLoc = SkyStoneLoc.RIGHT;
            DriveFieldRealtiveDistance(.5, 180, .4);
            distanceToDrive-=.3;

        }
        else
        {
            telemetry.log().add("CENTER");
            skyStoneLoc = SkyStoneLoc.CENTER;
            DriveFieldRealtiveDistance(.5, 180, .9);
            distanceToDrive-=.7;

        }
        telemetry.log().add("RIGHT SKYSTONE: " + colorSensorSideRight.red());
        telemetry.log().add("LEFT SKYSTONE: " + colorSensorSideLeft.red());

        DriveFieldRealtiveDistance(.25, 90, 1.35); //drive toward stone
        DriveFieldRealtiveDistance(.25,0,.9); //grab stone

        sleep(.5);

        DriveFieldRealtiveDistance(.5, 270, 5); //drive away from stone
        sleep(.5);
        setSpinner(.04);
        sleep(.5);
        setGrabber(1);
        sleep(.5);
        DriveFieldRealtiveDistance(.5,180, 7); //other side
        // setLiftLevel(1);
        // setFondationGrabber(1);
        // setSpinner(1);
        TurnPID(0,1);
        TurnPID(90,2);
        setGrabber(0);
        //setSpinner(0);
        //sleep(2.0);
        //setLiftLevel(0);

        DriveFieldRealtiveDistance(.5, 0,2); //go under skybridge
        sleep(1.0);

        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("wrote file");

    }
    boolean isSkystone(ColorSensor colorSensorToBeUsed)
    {
        return colorSensorToBeUsed.red() < 50;

    }
}
