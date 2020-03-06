package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;
import org.firstinspires.ftc.teamcode.util.MiniPID;

@TeleOp
@Config
@Disabled
public class LiftSlidePIDtest extends LinearOpMode {

    public static double P=0;
    public static double I=0;
    public static double D=0;

    public static int target = 0;

    private int realTarget;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.addData("Ready to start","\n");
        telemetry.update();

        MiniPID miniPID = new MiniPID(P,I,D);
        waitForStart();

        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        miniPID.setSetpoint(0);

        while (opModeIsActive())
        {
            realTarget = target + bot.getLiftStartingPos();
            miniPID.setP(P);
            miniPID.setI(I);
            miniPID.setD(D);
            miniPID.setSetpoint(realTarget);
            double actual = bot.lift.getCurrentPosition();
            double output = miniPID.getOutput(actual,realTarget);
            bot.lift.setPower(output);
        }
    }
}
