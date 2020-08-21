package org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.stateActions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

public class grabStone implements MarkerCallback {

    private DriveTrain6547State bot;
    
    public grabStone(DriveTrain6547State bot)
    {
        this.bot = bot;
    }

    @Override
    public void onMarkerReached() {
        bot.closeGrabber();
    }
}
