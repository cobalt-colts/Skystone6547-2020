package org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.stateActions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

public class MoveLiftUntilTarget implements MarkerCallback {


    DriveTrain6547State bot;
    int target;
    int leeway;

    public MoveLiftUntilTarget(DriveTrain6547State bot, int target, int leeway)
    {
        this.bot = bot;
        this.target = target;
        this.leeway = leeway;
    }
    public MoveLiftUntilTarget(DriveTrain6547State bot, int target)
    {
        this.bot = bot;
        this.target = target;
        leeway = 50;
    }
    @Override
    public void onMarkerReached() {
        bot.setLiftTargetPos(target);
        bot.setLiftLeeway(leeway);
        bot.setRunLift(true);
    }
}
