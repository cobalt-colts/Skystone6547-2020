package org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.stateActions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

public class Intake implements MarkerCallback {

    DriveTrain6547State bot;
    private double power;

    public Intake(DriveTrain6547State bot, double power) {
        this.bot = bot;
        this.power = power;
    }

    @Override
    public void onMarkerReached() {
        bot.intake(power);
    }
}
