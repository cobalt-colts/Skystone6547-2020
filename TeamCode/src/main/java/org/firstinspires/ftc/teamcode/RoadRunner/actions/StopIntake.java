package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class StopIntake implements MarkerCallback {

    DriveTrain6547State bot;

    public StopIntake(DriveTrain6547State bot) {
        this.bot = bot;
    }

    @Override
    public void onMarkerReached() {
        bot.stopIntake();
    }
}
