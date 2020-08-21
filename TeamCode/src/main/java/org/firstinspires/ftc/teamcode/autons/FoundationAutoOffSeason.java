package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveSpeeds;
import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED/Blue Foundation State", group = "auton")
public class FoundationAutoOffSeason extends LinearOpMode {

    private boolean isRed;
    private double yModifer; //all y values are multiplied by this in order to mirror this auton for blue side
    private double faceForwardDeg;
    private double faceBackwardDeg;

    DriveSpeeds driveSpeeds = new DriveSpeeds();

    public void runOpMode()
    {
        //make bot fast
        DriveConstants.BASE_CONSTRAINTS =  new DriveConstraints(
                60, 35, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0);

        DriveTrain6547State bot = new DriveTrain6547State(this); //the robot

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


        while (!gamepad1.x && !gamepad1.b && !isStopRequested())
        {
            //do nothing until button is pressed
        }
        if (gamepad1.x) //blue
        {
            telemetry.addData("You Choose the BLUE pill","");
            isRed = false;
            yModifer = -1; //mirror y axis
            faceForwardDeg = 270;
            faceBackwardDeg = 90;
        }
        else //red
        {
            telemetry.addData("You Choose the RED pill","");
            isRed = true;
            yModifer = 1;
            faceForwardDeg = 90;
            faceBackwardDeg = 270;
        }


        //set the position of the bot
        if (isRed) bot.setPoseEstimate(new Pose2d(35, -62,Math.toRadians(faceBackwardDeg)));
        else bot.setPoseEstimate(new Pose2d(35,62,Math.toRadians(faceBackwardDeg)));

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

        //go to foundatiom

        if (isRed) {

            // Let's let trajectories handle this instead
            //
            bot.followTrajectorySync(bot.trajectoryBuilder(true,driveSpeeds.slow)
                    .splineTo(new Pose2d(45, -31, Math.toRadians(faceForwardDeg)))
                    .build());
        } else {
            // Addedd
            bot.followTrajectorySync(bot.trajectoryBuilder(true, driveSpeeds.slow)
                   // .splineTo()
                    .splineTo(new Pose2d(45, 31, Math.toRadians(faceForwardDeg)))
                    .build());
        }

        // Turn to face foundation
        for (int i=0; i < 3; i++) {
            bot.turnRealtiveSync(Math.toRadians(faceBackwardDeg));
        }

        bot.openGrabber();

        bot.setFondationGrabber(1);     // Grab foundation


        //wait for foundation grabber to grab and grab the block


        sleep(1000);

        RobotLog.d("pulling foundation");

        //pull foundation
        if (isRed) {
            bot.turnSync(Math.toRadians(-40));
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(38)
                    .build());
        }
        else { //if blue
            bot.turnSync(Math.toRadians(40));
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(27)
                    .build());
        }

        //turn foundation toward wall

        if (isRed) bot.turnSync(Math.toRadians(-200));
        else bot.turnSync(Math.toRadians(170));

        //go to other stone
        bot.setFondationGrabber(0);

        RobotLog.d("Placing Foundation");
        if (isRed)
        {
            bot.driveBackward(6);
            bot.strafeLeft(11);
        }
        else
        {
            bot.driveBackward(10);
            bot.strafeRight(11);
        }



        RobotLog.d("Driving to under Skybridge");
        if (isRed) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    //.addTemporalMarker(2,new StopIntake(bot))
                    .splineTo(new Pose2d(5, -51, Math.toRadians(180)))
                    .build());
        }
        else {

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    //.addTemporalMarker(2,new StopIntake(bot))
                    .splineTo(new Pose2d(-2, 68, Math.toRadians(180)))
                    .build());
        }
        bot.intake(1);
        RobotLog.d("Driving to under Skybridge DONE");
        if (isRed)
        {
           bot.followTrajectorySync(bot.trajectoryBuilder(false,driveSpeeds.slow)
           .strafeLeft(10)
           .build());
        }
        else
        {
            bot.followTrajectorySync(bot.trajectoryBuilder(false,driveSpeeds.slow)
                    .strafeRight(10)
                    .build());
        }
        bot.stopIntake();

        if (isRed) bot.writeFile(bot.GYRO_ANGLE_FILE_NAME,Math.toDegrees(bot.getPoseEstimate().getHeading()) + 90);
        else bot.writeFile(bot.GYRO_ANGLE_FILE_NAME,Math.toDegrees(bot.getPoseEstimate().getHeading()) - 90);

        while(opModeIsActive()); // stop here


    }
}
