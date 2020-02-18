package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class Outtake implements Function0<Unit> {

    DriveTrain6547State bot;
    private double power;

    public Outtake(DriveTrain6547State bot, double power) {
        this.bot = bot;
        this.power = power;
    }

    @Override
    public Unit invoke() {
        bot.outtake(power);
        return Unit.INSTANCE;
    }
}
