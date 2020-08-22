package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

@Autonomous(name = "Drive Forward One Foot",group = "auton")
@Disabled
public class DriveForwardOneFootOffSeason extends LinearOpMode {

    boolean isRed;
    double faceForwardDeg;

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

        bot.setRunLift(true);

        bot.setLiftTargetPos(0);

        //bot.RetractGrabberSlide();

        bot.openGrabber();
        //get side of field by driver input

        telemetry.addData("Push B for RED, X for BLUE","");
        telemetry.update();


        while (!gamepad1.x && !gamepad1.b && !isStopRequested())
        {
            //do nothing until button is pressed
        }
        if (gamepad1.x) //blue
        {
            telemetry.addData("You Choose the BLUE pill","");
            RobotLog.d("BLUE SIDE SELECTED");
            isRed = false;
        }
        else //red
        {
            telemetry.addData("You Choose the RED pill","");
            isRed = true;
            faceForwardDeg = 0;
        }


        //set the position of the bot
        if (isRed) bot.setPoseEstimate(new Pose2d(-36, -62,Math.toRadians(180)));
        else bot.setPoseEstimate(new Pose2d(-36,62,Math.toRadians(0)));

        telemetry.addData("Ready to start","");
        telemetry.update();
        telemetry.log().add("Ready to start");
        RobotLog.v("Waiting For Start...");

        bot.grabberSlide.setPower(-.10);
        while (!isStopRequested() && !isStarted() && !bot.isTouchSensorPressed())
        {
            bot.setLiftPower(-1);
        }
        bot.grabberSlide.setPower(0);
        bot.setLiftPower(0);
        bot.zeroEncoder(bot.lift);

        RobotLog.v("Lift lowered...");

        while (!isStarted() && !isStopRequested())
        {
            if (!bot.isTouchSensorPressed()) {
                bot.setLiftToTargetPos(0, 100);
            }
            telemetry.addData("Lift Pos",bot.lift.getCurrentPosition());
            telemetry.update();
        }
        telemetry.log().add("Starting");
        RobotLog.v("Starting");

        telemetry.log().add("Ready to start");
        waitForStart();

        //park under SkyBridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .forward(12)
        .build());

        //save gyro angle
        if (isRed) bot.writeFile(bot.GYRO_ANGLE_FILE_NAME,Math.toDegrees(bot.getPoseEstimate().getHeading()) + 90);
        else bot.writeFile(bot.GYRO_ANGLE_FILE_NAME,Math.toDegrees(bot.getPoseEstimate().getHeading()) - 90);

        while (opModeIsActive())
        {
            bot.outputTelemetry();
        }
    }
}
