package org.firstinspires.ftc.teamcode.testing.circles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
This class is a circle class intended for use for the robot to drive into a circle

This class was created as a navigational prototype to get the robot to drive in circles.
Needs more posishing to work, however progress on this is halted because we need a working autonomous
and tele-op (ultilizing last years working code) first.
 */
@Disabled
public class Circle {
    public double circumference=0;
    public double area=0;
    private double x;
    private double y;
    private double angleFromCenter=0;
    public double r;
    public Circle(double raduis)
    {
        area = Math.PI*Math.pow(r,2);
        circumference = Math.PI * (2*r);
        r=raduis;
    }
    public void updateCords(double distanceTraveled)
    {
        double distanceFrac = distanceTraveled/circumference;
        angleFromCenter = 180*distanceFrac;
        // x = Math.cos(Math.toRadians(angleFromCenter)) * r;
        //y = Math.sin(Math.toRadians(angleFromCenter)) * r;
    }
    public double getAngleFromCenter()
    {
        return angleFromCenter;
    }
    public double getAngleToDrive(double angle)
    {
        return angle+90;
    }
    public double getX()
    {
        return x;
    }
    public double getY()
    {
        return y;
    }
}