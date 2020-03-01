package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;

@Autonomous(name = "Drive Forward One Foot",group = "auton")
public class DriveForwardOneFoot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547State bot = new DriveTrain6547State(this); //the bot
          /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */
        //bot.setLiftTargetPos(bot.liftStartingPos);
        //bot.setRunLift(true);

        while (!isStopRequested() && !isStarted() && !bot.isTouchSensorPressed())
        {
            bot.setLiftPower(-1);
        }
        bot.setLiftPower(0);
        bot.zeroEncoder(bot.lift);

        telemetry.log().add("Ready to start");
        //waitForStart();

        //park under SkyBridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(12)
        .build());

        //save gyro angle
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        while (opModeIsActive())
        {
            bot.outputTelemetry();
        }
    }
}
