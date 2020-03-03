package org.firstinspires.ftc.teamcode.RoadRunner.actions;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

public class MoveGrabberSlideTime implements MarkerCallback {

    private DriveTrain6547State bot;
    private double power;
    private long milliseconds;

    public MoveGrabberSlideTime(DriveTrain6547State bot, double power, long milliseconds) {
        this.bot = bot;
        this.power = power;
        this.milliseconds = milliseconds;
    }

    public MoveGrabberSlideTime(DriveTrain6547State bot, long milliseconds) {
        this.bot = bot;
        this.milliseconds = milliseconds;
        power=1;
    }

    @Override
    public void onMarkerReached() {
        bot.moveGrabberSlideForTime(power, milliseconds);
    }
}
