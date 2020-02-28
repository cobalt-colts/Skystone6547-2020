package org.firstinspires.ftc.teamcode.oldPrograms.oldAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet1.SkyStone6547;

/*
  This auton pulls the foundation to the build site and parks under the skybridge for the BLUE side
  Note: main difference between red and blue side is the robot moves in opposite directions to move to other side of the field
 */
@Autonomous(name = "BLUE Foundation Auton")
@Disabled
public class BlueFondationAuton extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();
        angleZzeroValue=180;

        zeroEncoders();
        telemetry.log().add("ready to start");

        waitForStart();

        //setLiftLevel(0);

        DriveFieldRealtiveDistance(.3, 90, 3); //drive toward stone

        sleep(.5);

        setFondationGrabber(.6);

        sleep(1.0);

        DriveFieldRealtiveDistance(.3, 270, 9); //drive away from stone
        setFondationGrabber(0);
        //DriveFieldRealtiveDistance(.5, 0, 1);

        sleep(.5);

        DriveFieldRealtiveDistance(.5, 0,4); //park to skybridge

        sleep(.5);

        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("wrote file");

    }
}
