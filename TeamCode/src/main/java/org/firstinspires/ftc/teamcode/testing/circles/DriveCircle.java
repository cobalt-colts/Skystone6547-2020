package org.firstinspires.ftc.teamcode.testing.circles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet1.SkyStone6547;

/*
This class is part of the idea to get the robot to drive into a circle.

This use the Circle class in order to determine the circle.
However, this class in incomplete and going to be obsolete.
Circular driving will be moved to DriveInACircle.java
 */
@Autonomous
@Disabled
public class DriveCircle extends SkyStone6547 {

    public void runOpMode()
    {
        INIT(hardwareMap);

        initIMU();

        Circle circle = new Circle(5);
        zeroEncoders();
        telemetry.log().add("ready to start");

        waitForStart();

        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;

        double i=0;
        while (opModeIsActive())
        {
            i+=1;
            //circle.updateCords(Math.abs(averageDrivetrainEncoder()));
            // double angleToDrive = circle.getAngleToDrive();
            double distanceFrac =(averageDrivetrainEncoder()/(Math.PI*(25/2)));
            double angleFromCenter = 180*distanceFrac;
            DriveFieldRealtiveSimple(.5, angleFromCenter+90);
            telemetry.addData("average encoder", averageDrivetrainEncoder());
            telemetry.update();
        }

        //driveCircle(circle, .5, 0,180);
    }
    void driveCircle(Circle circle, double pow, double fromAngle, double toAngle)
    {
        double travelDist = (180/(toAngle - fromAngle))*(circle.circumference/2);
        double averageEncoder;
        zeroEncoders();
        while (opModeIsActive() && averageDrivetrainEncoder() < travelDist)
        {
            averageEncoder=averageDrivetrainEncoder();

        }
        stopRobot();
    }
    // todo: write your code here
}