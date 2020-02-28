package org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

public class DriveSpeeds {

    DriveConstraints slow = new DriveConstraints(40, 20, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0);

    DriveConstraints medium = new DriveConstraints(60,35,0.0,
            Math.toRadians(180), Math.toRadians(180),0.0);

    DriveConstraints fast = new DriveConstraints(80,50,0.0,
            Math.toRadians(180),Math.toRadians(180),0.0);

    DriveConstraints sonicSpeed = new DriveConstraints(110,70,0.0,
            Math.toRadians(180),Math.toRadians(180),0.0);
}
