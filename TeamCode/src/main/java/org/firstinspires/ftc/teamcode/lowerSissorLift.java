package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveTrain6547Offseason;

@TeleOp
public class lowerSissorLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain6547Offseason bot = new DriveTrain6547Offseason(this); //the robot

        /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */

        bot.setRunLift(true);

        bot.setLiftTargetPos(0);

        //bot.RetractGrabberSlide();

        bot.openGrabber();
        //get side of field by driver input

        telemetry.addData("Push B for RED, X for BLUE","");
        telemetry.update();

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

        while (opModeIsActive())
        {
            if (!bot.isTouchSensorPressed()) {
                bot.setLiftToTargetPos(0, 100);
            }
            telemetry.addData("Lift Pos",bot.lift.getCurrentPosition());
            telemetry.update();
        }
        telemetry.log().add("Starting");
        RobotLog.v("Starting");

    }
}
