package org.firstinspires.ftc.teamcode.oldPrograms.usedAtState;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

@TeleOp(name = "Calibate Sissour Lift", group = "cali")
@Disabled
public class CalibabateSissourLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this);

        telemetry.log().add("ready to start");
        waitForStart();
        telemetry.log().add("Use the left stick and the right stick to move the lift");
        bot.setBulkReadOff();
        
        

        while (opModeIsActive())
        {
            bot.updateGamepads();
            bot.lift.setPower(-gamepad1.right_stick_y);
            if (bot.a1.onPress()) //if A is pressed, write the auton starting pos
            {
                bot.zeroEncoder(bot.lift);
                //TODO: Remove below statement
                bot.writeFile(bot.LIFT_STARTING_POS_FILE_NAME,bot.lift.getCurrentPosition());
            }
            if (bot.b1.onPress()) //if B is pressed, write the max
            {
                bot.writeFile(bot.LIFT_MAX_FILE_NAME,bot.lift.getCurrentPosition());
            }
            telemetry.addData("lift position", bot.lift.getCurrentPosition());
            telemetry.update();
        }
    }
}
