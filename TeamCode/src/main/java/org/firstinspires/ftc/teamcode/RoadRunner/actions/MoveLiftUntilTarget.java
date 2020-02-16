package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class MoveLiftUntilTarget implements Function0<Unit> {


    DriveTrain6547 bot;
    int target;
    int leeway;

    public MoveLiftUntilTarget(DriveTrain6547 bot, int target, int leeway)
    {
        this.bot = bot;
        this.target = target;
        this.leeway = leeway;
    }
    public MoveLiftUntilTarget(DriveTrain6547 bot, int target)
    {
        this.bot = bot;
        this.target = target;
        leeway = 50;
    }
    @Override
    public Unit invoke() {
        bot.setLiftTargetPos(target);
        bot.setLiftLeeway(leeway);
        bot.setRunLift(true);
        return Unit.INSTANCE;
    }
}
