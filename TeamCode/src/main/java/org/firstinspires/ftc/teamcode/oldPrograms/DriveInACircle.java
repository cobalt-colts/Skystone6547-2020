package org.firstinspires.ftc.teamcode.OldPrograms;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

/*
Another class to drive in circle.  Unlike the other classes, this class does not use the Circle class
in favor of just having a method because it will be easier to work with.

In addition, this class experiments with driving in a ellipse, the next step up from a circle.
Currently, progress on Ellipse driving is halted because I need to know Calc and the robot is driving to straight to be
considered an ellipse.  And I should get a working auton and tele-op with last years code first.
 */
public class DriveInACircle extends SkyStone6547 {

    @Override
    public void runOpMode() {
        INIT(hardwareMap);

        initIMU();

        zeroEncoders();
        telemetry.log().add("ready to start");
        waitForStart();
    }

    void driveCircle(double pow, double r, double toAngle, boolean left)
    {
        zeroEncoders();
        double midDist = (r-.75)* Math.PI * (toAngle/180);
        double leftDist = r * Math.PI * (toAngle/180);
        double rightDist = (r-1.5) * Math.PI * (toAngle/180);

        double leftPow = 1; // leftDist/leftDist
        double rightPow = rightDist/leftDist;
        if (!left)
        {
            leftPow+=rightPow; //swap left and right, no temp variable because I have unparalleled intellect (and Google)
            rightPow=leftPow-rightPow;
            leftPow-=rightPow;
        }
        leftPow*=pow;
        rightPow*=pow;
        while (opModeIsActive() && averageDrivetrainEncoder() <= midDist)
        {
            steerRobot(leftPow, rightPow);
        }
        stopRobot();
    }
    void driveEllipse(double pow,double a, double b, double toAngle, boolean left)
    {
        driveEllipse(pow,a,b,0,toAngle, left);
    }
    void driveEllipse(double pow, double a, double b, double fromAngle, double toAngle, boolean left)
    {
        double midDist = (getEllipseCircumference(a-.75,b-.75)/2)*((toAngle-fromAngle)/360);
        double leftDist = (getEllipseCircumference(a, b)/2)*((toAngle-fromAngle)/360);
        double rightDist = (getEllipseCircumference(a-1.5, b-1.5)/2)*((toAngle-fromAngle)/360);

        double leftPow = 1; // leftDist/leftDist
        double rightPow = rightDist/leftDist;
        if (!left) //if not going left, reflect the powers.  Goes oppoiste direction
        {
            leftPow+=rightPow; //swap left and right, no temp variable because I can
            rightPow=leftPow-rightPow;
            leftPow-=rightPow;
        }
        leftPow*=pow;
        rightPow*=pow;
        while (opModeIsActive() && averageDrivetrainEncoder() <= midDist)
        {
            steerRobot(leftPow, rightPow);
        }
        stopRobot();
    }
    double getEllipseCircumference(double a, double b)
    {
        double c = (Math.PI * (a+b))*(3*(Math.pow(a-b,2)/(Math.pow(a+b,2)*(Math.sqrt((-3*(Math.pow(a-b,2)/Math.pow(a+b,2)))+4)+10)))+1);
        return c;
    }

}
