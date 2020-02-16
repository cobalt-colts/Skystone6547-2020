package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class Intake implements Function0<Unit> {

    DriveTrain6547 bot;
    private double power;

    public Intake(DriveTrain6547 bot, double power) {
        this.bot = bot;
        this.power = power;
    }

    @Override
    public Unit invoke() {
        bot.intake(power);
        return Unit.INSTANCE;
    }
}
