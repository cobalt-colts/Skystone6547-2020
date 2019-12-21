package org.firstinspires.ftc.teamcode.MuiltthreadingTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/*
This is a class that calculated the robot's x and y position based on where it goes
using the encoders on the wheels.  This class is meant to be a thread so the position is constantly updated, no matter what

This was a failure.

this will not record the position accurately if the robot strafes.
Can't figure out how to use the imu (gyro) in multithreading without crashing the program.

 */
@Disabled
public class CalcPos implements Runnable {


    double xDist=0;
    double yDist=0;
    double oldDist=0;

    public boolean isRunning;
    DcMotor[] motors; //motors on the drivetrain
    //Orientation angles; //the gyro angle object.  Only thing used is angles.firstAngle;
    public CalcPos(DcMotor[] motorsToRun)
    {
        motors=motorsToRun;
        //angles= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }
//    public void start()
//    {
//        for (DcMotor motor : motors){
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
    public double getXDist()
    {
        return xDist;
    }
    public double getYDist()
    {
        return yDist;
    }
    public void run()
    {
        while (isRunning)
        {
            try
            {
                updateDistance();
            }
            catch (Exception e)
            {

            }
        }
    }
    public void stop()
    {
        isRunning = false;
    }
//    public double getAngle()
//    {
//        return angles.firstAngle;
//    }
    private void updateDistance()
    {
        double dist = avg(motors)-oldDist; //grab the difference between the distance from the previous code update and the current code update
        xDist+=dist;
        yDist+=dist;
        oldDist+=dist;

    }
    private double avg(DcMotor[] motors)
    {
        //compare all the motors positions (from encoders) and average them out.
        double motorpos=0;
        for (DcMotor motor : motors)
        {
            motorpos+=motor.getCurrentPosition();
        }
        return motorpos/motors.length;
    }
}