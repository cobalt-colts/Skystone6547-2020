package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

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
