package org.firstinspires.ftc.teamcode.testing.circles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet1.SkyStone6547;

/*
Another class to drive in circle.  Unlike the other classes, this class does not use the Circle class
in favor of just having a method because it will be easier to work with.

In addition, this class experiments with driving in a ellipse, the next step up from a circle.
Currently, progress on Ellipse driving is halted because I need to know Calc and the robot is driving to straight to be
considered an ellipse.  And I should get a working auton and tele-op with last years code first.
 */
@Autonomous
@Disabled
public class DriveTrainACircle extends SkyStone6547 {

    double deltaX=0;
    double deltaY=0;
    double oldDist=0;

    @Override
    public void runOpMode() {
        INIT(hardwareMap);

        initIMU();

        zeroEncoders();
        telemetry.log().add("ready to start");
        telemetry.log().add("average encoder: " + averageDrivetrainEncoder());
        //telemetry.log().add("mid dist: " + ((2-.75)* Math.PI * (180/180)));
        waitForStart();

        driveCircle(.25, 1.4, 60, true, true);
        DriveFieldRealtiveDistance(.25, 35, 1);
        //TurnPID(20,1);
        driveCircle(.25, 1.6, 45, false, true);
        // driveEllipse(.25,1,1,180,true,true);


    }

    void driveCircle(double pow, double r, double toAngle, boolean left, boolean forward)
    {
        zeroEncoders();
        double midDist = (r-.75)* Math.PI * (toAngle/180)*(encoderTicksPerRotation/circumferenceOfWheel)*12;
        double leftDist = r * Math.PI * (toAngle/180);
        double rightDist = (r-1.5) * Math.PI * (toAngle/180);

        double leftPow = 1; // leftDist/leftDist
        double rightPow = rightDist/leftDist;
        if (!left)
        {
            leftPow+=rightPow; //swap left and right, no temp variable because I can
            rightPow=leftPow-rightPow;
            leftPow-=rightPow;
        }
        if (forward)
        {
            leftPow=-Math.abs(leftPow);
            rightPow=-Math.abs(rightPow);
        }
        leftPow*=pow;
        rightPow*=pow;
        while (opModeIsActive() && averageDrivetrainEncoder() <= midDist)
        {
            steerRobot(leftPow, rightPow);
        }
        stopRobot();
    }
    double getCircleCircumfrence(double r)
    {
        return (r*2) * Math.PI;
    }
    void driveEllipse(double pow,double a, double b, double toAngle, boolean left, boolean forward)
    {
        driveEllipse(pow,a,b,0,toAngle, left, forward);
    }
    void driveEllipse(double pow, double a, double b, double fromAngle, double toAngle, boolean left, boolean forward)
    {
        double midDist = (getEllipseCircumference(a-.5,b-.5))*((toAngle-fromAngle)/360)*12*205;
        double leftDist = (getEllipseCircumference(a, b))*((toAngle-fromAngle)/360);
        double rightDist = (getEllipseCircumference(a-1, b-1))*((toAngle-fromAngle)/360);

        double leftPow = 1; // leftDist/leftDist
        double rightPow = rightDist/leftDist;
        leftPow*=pow; //left pow is a constant.
        resetDetlaPos();
        while (opModeIsActive() && averageDrivetrainEncoder() <= midDist)
        {
            updateDistance();
            double r = getCircleRaduis(getEllipseCurvature(deltaX));
            double leftCir = getCircleCircumfrence(r+.5);
            double rightCir = getCircleCircumfrence(r-.5);
            rightPow = rightCir/leftCir;
            rightPow*=pow;
            if (!left) //if not going left, reflect the powers.  Goes opposite direction
            {
                leftPow = rightPow;
                rightPow = pow;
            }
            if (!forward)
            {
                leftPow=-Math.abs(leftPow);
                rightPow=-Math.abs(rightPow);
            }
            steerRobot(leftPow, rightPow);
            telemetry.addData("X", deltaX);
            telemetry.addData("Y", deltaY);
            telemetry.addData("left pow", LeftFront.getPower());
            telemetry.addData("right pow", RightBack.getPower());
            telemetry.update();
        }
        stopRobot();
    }
    double getCircleRaduis(double angle)
    {
        return 1/angle;
    }
    double getEllipseCurvature(double x)
    {
        double c = 2500*Math.abs(1/Math.pow(Math.abs((116*Math.pow(x, 2))-625), 1.5));

        return c;
    }
    double getEllipseCurvature(double a, double b, double x)
    {
        double d1 = (Math.pow(b,2) * x)/(Math.pow(Math.abs(((Math.pow(x,2)*Math.pow(b,2))/Math.pow(a,2))-Math.pow(b,2)),.5)*Math.pow(a,2));
        double d2 = -(Math.pow(b,4))/(Math.pow(Math.abs(((Math.pow(x,2)*Math.pow(b,2))/Math.pow(a,2))-Math.pow(b,2)),1.5)*Math.pow(a,2));
        double c = Math.abs(d2)/Math.pow(Math.abs(1+(Math.pow(d1,2))),1.5);
        return c;
    }
    private void updateDistance()
    {
        double dist = averageDrivetrainEncoder()-oldDist;
        deltaX+=dist*Math.cos(Math.toRadians(angles.firstAngle));
        deltaY+=dist*Math.sin(Math.toRadians(angles.firstAngle));
        oldDist+=dist;

    }
    public void resetDetlaPos()
    {
        deltaY=0;
        deltaX=0;
    }
    double getEllipseAngle(double a, double b, double x, double leeway)
    {
        double x1 = x+leeway;
        double x2 = x-leeway;
        double y1 = calcy(a,b,x1);
        double y2 = calcy(a,b,x2);
        return Math.toDegrees(Math.atan((y2-y1)/(x2-x1)));
    }
    double calcy(double a, double b, double x)
    {
        return Math.sqrt(((Math.pow(x,2)/Math.pow(a,2))*Math.pow(b,2))-Math.pow(b,2));
    }
    double getEllipseCircumference(double a, double b)
    {
        double c = (Math.PI * (a+b))*(3*(Math.pow(a-b,2)/(Math.pow(a+b,2)*(Math.sqrt((-3*(Math.pow(a-b,2)/Math.pow(a+b,2)))+4)+10)))+1);
        return c;
    }

}