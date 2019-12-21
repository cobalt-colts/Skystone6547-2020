package org.firstinspires.ftc.teamcode.OldPrograms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

/*
Prototype using multithreading in order to move the linear side and arm when the robot is moving and such.

Progress halted for now because multithreading is complex
 */
@Disabled
public class LinearSlideMover extends SkyStone6547 implements Runnable {

    private DcMotor lift;
    private Servo spinner;
    private Servo grabber;
    private int level;
    private double spinPos;

    private boolean isRunning=false;

    public LinearSlideMover(DcMotor lift_, Servo spinner_, Servo grabber_, int level_, double spinPos_)
    {
        lift = lift_;
        spinner = spinner_;
        grabber = grabber_;
        level = level_;
        spinPos = spinPos_;
    }
    public void run()
    {
        liftAndSpin(level,spinPos);
    }
    public void liftAndSpin(int level, double spinPos)
    {
        setLiftLevel(level);
        setSpinner(spinPos);
        sleep(.5);
    }

    public void runOpMode(){}

}