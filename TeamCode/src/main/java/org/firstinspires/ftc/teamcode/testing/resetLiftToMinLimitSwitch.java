package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

@TeleOp(group = "cali")
@Disabled
public class resetLiftToMinLimitSwitch extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive() && !bot.isTouchSensorPressed())
        {
            bot.setLiftPower(-1);
        }
        bot.setLiftPower(0);
        bot.zeroEncoder(bot.lift);
        RobotLog.d("Done calibrating lift");
        sleep(500);
    }
}
