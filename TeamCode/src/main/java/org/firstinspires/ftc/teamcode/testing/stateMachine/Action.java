package org.firstinspires.ftc.teamcode.testing.stateMachine;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInQualifier.SkyStone6547Qualifter;

@Disabled
public class Action {

    public static SkyStone6547Qualifter bot;
    private DcMotor[] motors;
    private int target;
    private double power;
    private boolean done = false;
    //private boolean important = true;
    public Action(SkyStone6547Qualifter bot, DcMotor[] motors, double power, int target)
    {
        this.bot = bot;
        this.motors = motors;
        this.power = power;
        this.target = target;
        for (int i =0; i < motors.length; i++)
        {
            motors[i] = bot.zeroEncoder(motors[i]);
        }
    }
    public Action(SkyStone6547Qualifter bot, Mode mode, double power, double target)
    {

    }
    public boolean isDone()
    {
        return done;
    }
    public void runState() {
        if (avgEncoders() < Math.abs(target)) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(power);
            }
        }
        else
        {
            done = true;
        }
    }
    public double avgEncoders()
    {
        int num=0;
        for (DcMotor motor : motors)
        {
            num+=Math.abs(motor.getCurrentPosition());
        }
        return num/motors.length;
    }
}
enum Mode
{
    NORMAL, DRIVE_FIELD_RELATIVE
}
