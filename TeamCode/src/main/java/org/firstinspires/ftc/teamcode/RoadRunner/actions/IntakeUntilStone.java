package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class IntakeUntilStone implements Function0<Unit> {

    private DriveTrain6547 bot;

    public IntakeUntilStone(DriveTrain6547 bot) {
        this.bot = bot;
    }


    @Override
    public Unit invoke() {
        bot.setRunIntakeUntilStone(true);
        return Unit.INSTANCE;
    }
}
