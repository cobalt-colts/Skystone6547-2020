package org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.stateActions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

public class IntakeUntilStone implements MarkerCallback {

    private DriveTrain6547State bot;

    public IntakeUntilStone(DriveTrain6547State bot) {
        this.bot = bot;
    }

    @Override
    public void onMarkerReached() {
        bot.setRunIntakeUntilStone(true);
    }
}
