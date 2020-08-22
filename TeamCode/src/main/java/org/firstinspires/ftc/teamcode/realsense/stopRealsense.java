package org.firstinspires.ftc.teamcode.realsense;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class stopRealsense extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        new DriveTrain6547Realsense(this).stopRealsense();
    }
}
