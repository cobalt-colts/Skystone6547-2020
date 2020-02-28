package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class Outtake implements MarkerCallback {

    DriveTrain6547State bot;
    private double power;

    public Outtake(DriveTrain6547State bot, double power) {
        this.bot = bot;
        this.power = power;
    }

    @Override
    public void onMarkerReached() {
        bot.outtake(power);
    }
}
