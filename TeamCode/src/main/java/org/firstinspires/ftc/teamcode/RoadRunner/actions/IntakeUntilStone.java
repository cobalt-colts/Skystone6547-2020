package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class IntakeUntilStone implements Function0<Unit> {

    private DriveTrain6547State bot;

    public IntakeUntilStone(DriveTrain6547State bot) {
        this.bot = bot;
    }


    @Override
    public Unit invoke() {
        bot.setRunIntakeUntilStone(true);
        return Unit.INSTANCE;
    }
}
