package org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.stateActions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

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
