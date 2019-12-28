package org.firstinspires.ftc.teamcode.oldPrograms.oldAutons.Meet2Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

/*
    This auton pulls the foundation to the build site and parks under the skybridge for the RED side
 */
@Autonomous(name = "BLUE Foundation Auton meet2")
@Disabled
public class BlueFoundationAuton extends SkyStone6547Meet2 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        angleZzeroValue=180;

        zeroEncoders();
        telemetry.log().add("ready to start");

        waitForStart();
        
        TurnPID(180,1);

        //setLiftLevel(0);
        
        DriveFieldRealtiveDistance(.3,180,1);
        TurnPID(180,1);

        DriveFieldRealtiveDistance(.3, 90, 3.1); //drive toward stone
        TurnPID(180,1);

        sleep(.5);

        setFondationGrabber(.9);

        sleep(1.0);

        DriveFieldRealtiveDistance(.3, 270, 4); //drive away from stone
        sleep(.5);
        TurnPID(90,5);
        sleep(.5);
        setFondationGrabber(0);
        //DriveFieldRealtiveDistance(.5, 0, 1);

        sleep(.5);
        
        DriveFieldRealtiveDistance(.3,270,.5);

        DriveFieldRealtiveDistance(.5, 0,4); //park to skybridge

        setFondationGrabber(0);

        sleep(.5);

        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("wrote file");

    }
}
