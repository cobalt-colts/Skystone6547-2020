package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class StopIntake implements Function0<Unit> {

    DriveTrain6547 bot;

    public StopIntake(DriveTrain6547 bot) {
        this.bot = bot;
    }

    @Override
    public Unit invoke() {
        bot.stopIntake();
        return Unit.INSTANCE;
    }
}
