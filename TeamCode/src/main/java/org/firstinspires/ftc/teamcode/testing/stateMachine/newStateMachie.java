package org.firstinspires.ftc.teamcode.testing.stateMachine;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInQualifier.SkyStone6547Qualifter;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class newStateMachie {

    SkyStone6547Qualifter bot;
    private int state = 0;
    int maxIndex=0;
    List<Boolean> doneIsh;

    public newStateMachie(SkyStone6547Qualifter bot)
    {
        this.bot = bot;
    }

    public void doState()
    {
        switch (state)
        {
            case 0:
                moveMotor(bot.LeftFront,.4,2000,0);
                moveMotor(bot.LeftBack, .4, 2000,1);
                moveMotor(bot.RightFront, .4, 2000,2);
                moveMotor(bot.RightBack, .4, 2000,3);
                checkStates();
                break;
            case 1:
                moveMotor(bot.LeftFront,-.4,2000,0);
                moveMotor(bot.LeftBack, -.4, 2000,1);
                moveMotor(bot.RightFront, -.4, 2000,2);
                moveMotor(bot.RightBack, -.4, 2000,3);
                checkStates();
                break;
            case 2:
                bot.stopRobot();
                break;
        }
    }
    public void moveMotor(DcMotor motor, double power, int target, int index)
    {
        if (index > maxIndex) maxIndex = index;
        motor.setPower(power);
        if (Math.abs(motor.getCurrentPosition()) > Math.abs(target))
        {
            setDone(index, false);
            bot.outputTelemetry();
        }
        else
        {
            motor.setPower(0);
            setDone(index, true);

        }
    }
    public void setDone(int index, boolean bool)
    {
        try {
            doneIsh.set(index, bool);
        }
        catch (Exception e)
        {

        }
    }
    public void resetDone()
    {
        doneIsh = new ArrayList<>();
    }
    public boolean allDone()
    {
        for (Boolean bool : doneIsh)
        {
            if (!bool) return false;
        }
        return true;
    }
    public void checkStates()
    {
        try {
            if (maxIndex != doneIsh.size()) {
                doneIsh = new ArrayList<>();
                for (int i = 0; i <= maxIndex; i++) {
                    doneIsh.add(false);
                }
                if (allDone()) {
                    bot.zeroEncoders();
                    resetDone();
                    state++;
                }
            }
            bot.opMode.telemetry.update();
        } catch (Exception e)
        {

        }
    }
}
