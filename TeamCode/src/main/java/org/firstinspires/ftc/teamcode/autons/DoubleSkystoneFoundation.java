package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.actions.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.IntakeUntilStone;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "RED/Blue double skystone Foundation State", group = "auton")
public class DoubleSkystoneFoundation extends LinearOpMode {

    private boolean isRed;
    private double yModifer; //all y values are multiplied by this in order to mirror this auton for blue side

    public void runOpMode()
    {
        //make bot fast
        DriveConstants.BASE_CONSTRAINTS =  new DriveConstraints(
                80, 40, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0);

        DriveTrain6547State bot = new DriveTrain6547State(this); //the robot

        /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */

        bot.setLiftTargetPos(bot.liftStartingPos);
        bot.setRunLift(false);

        bot.openGrabber();

        //get side of field by driver input

        telemetry.addData("Push B for RED, X for BLUE","");
        telemetry.update();


        while (!gamepad1.x && !gamepad1.b)
        {
            //do nothing until button is pressed
        }
        if (gamepad1.x) //blue
        {
            isRed = false;
            yModifer = -1; //mirror y axis
        }
        else //red
        {
            isRed = true;
            yModifer = 1;
        }


        //set the position of the bot
        bot.setPoseEstimate(new Pose2d(-36, -62*yModifer,Math.toRadians(90)));

        telemetry.addData("Ready to start","");
        telemetry.update();
        while (!isStarted())
        {
            bot.runAtAllTimes();
        }

        //Go to Skystones
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .splineTo(new Pose2d(-35,-31*yModifer,Math.toRadians(90)))
        .build());

        sleep(500);
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//                .strafeTo(new Vector2d(-36,-36))
//                .build());
        //scan stones

        if (bot.isSkystone(bot.colorSensorSideLeft))
        {
            // ---SKYSTONE LEFT---
            telemetry.log().add("LEFT");
            bot.skyStoneLoc = SkyStoneLoc.LEFT;

            if (isRed) { //if red
                bot.followTrajectorySync(bot.trajectoryBuilder()
                        .back(4)
                        .addMarker(new IntakeUntilStone(bot))
                        .strafeTo(new Vector2d(-52, -30))
                        .build());
            }
            else //if blue (this is the same path as going right for RED)
            {
                bot.followTrajectorySync(bot.trajectoryBuilder()
                        .back(4)
                        .addMarker(new IntakeUntilStone(bot))
                        .strafeTo(new Vector2d(-18,30))
                        .build());
            }

        }
        else if (bot.isSkystone(bot.colorSensorSideRight))
        {
            //---SKYSTONE RIGHT---
            telemetry.log().add("RIGHT");
            bot.skyStoneLoc = SkyStoneLoc.RIGHT;

            if (isRed) {
                //drive back a bit and strafe right
                bot.followTrajectorySync(bot.trajectoryBuilder()
                        .back(4)
                        .addMarker(new IntakeUntilStone(bot))
                        .strafeTo(new Vector2d(-18, -30))
                        .build());
            }
            else //if blue (path is same as left for RED)
            {
                bot.followTrajectorySync(bot.trajectoryBuilder()
                        .back(4)
                        .addMarker(new IntakeUntilStone(bot))
                        .strafeTo(new Vector2d(-52, 30))
                        .build());
            }
        }
        else //center (same for both colors)
        {
            //open intake
            bot.setRunIntakeUntilStone(true);
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(-36,-20,Math.toRadians(90)))
            .build());
            //---SKYSTONE CENTER---
            //go back a bit
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

        }

        if (bot.getRunIntakeUntilStone())
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .forward(12)
            .build());
        }

        //open intake

        //drive forward and intake stone
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//                .forward(18)
//        .build());

        //spline to under the skybridge
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,-40*yModifer,Math.toRadians(180)))
                .splineTo(new Pose2d(35,-40*yModifer,Math.toRadians(180)))
                .splineTo(new Pose2d(45,-21*yModifer,Math.toRadians(270)))
                .build());

        bot.setFondationGrabber(1);

        sleep(500);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(20)
                .splineTo(new Pose2d(33, -60*yModifer, Math.toRadians(180)))
                .build());

        bot.turnSync(Math.toRadians(-200));

        //go to other stone

        bot.setFondationGrabber(0);

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(24)
                .addMarker(1,new Intake(bot,1))
        .splineTo(new Pose2d(0,-40*yModifer,Math.toRadians(180)))
        .build());

        bot.setRunIntakeUntilStone(true);

        double constX= -25;
        double constY = -34;

        //prepare to grab the other Skystone
        if ((bot.skyStoneLoc == SkyStoneLoc.RIGHT && isRed) || (bot.skyStoneLoc == SkyStoneLoc.LEFT && !isRed))
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(constX,constY*yModifer,Math.toRadians(180)))
            .splineTo(new Pose2d(-45,-22*yModifer,Math.toRadians(135)))
            .build());
        }
        else if (bot.skyStoneLoc == SkyStoneLoc.CENTER)
        {

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(constX-5,constY*yModifer,Math.toRadians(180)))
            .splineTo(new Pose2d(-50,-22*yModifer, Math.toRadians(130)))
            .build());


        }
        else if ((bot.skyStoneLoc == SkyStoneLoc.LEFT && isRed) || (bot.skyStoneLoc == SkyStoneLoc.RIGHT && !isRed))
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(constX,constY*yModifer,Math.toRadians(180)))
                    .splineTo(new Pose2d(-60,-22*yModifer,Math.toRadians(140)))
                    .build());
        }

        //if the bot has not grabbed a stone yet, go forward a bit
        if (bot.getRunIntakeUntilStone()) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(8)
                    .build());
        }


        //deliver second stone
       bot.followTrajectorySync(bot.trajectoryBuilder()
               .reverse()
       .splineTo(new Pose2d(0,-36*yModifer,Math.toRadians(180)))
               .addMarker(1.5,new Intake(bot,1))
               .splineTo(new Pose2d(-8,-36*yModifer,Math.toRadians(180)))
       .build());

        //park under skybridge

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeTo(new Vector2d(0,-33*yModifer))
       .build());

       //strafe to give more room to allience partner

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeRight(10)
       .build());

       bot.stopIntake();


        //save gyro angle
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, bot.getIMUAngle());

        telemetry.log().add("wrote file at " + bot.getIMUAngle() + " degrees");
        //run until auton is over to keep the scissor lift down
        while (opModeIsActive()) {
            bot.outputTelemetry();
        }

    }
}
