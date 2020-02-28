package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RoadRunner.actions.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.IntakeUntilStone;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.StopIntake;
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
    private double faceForwardDeg;
    private double faceBackwardDeg;

    public void runOpMode()
    {
        //make bot fast
        DriveConstants.BASE_CONSTRAINTS =  new DriveConstraints(
                70, 40, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0);

        DriveTrain6547State bot = new DriveTrain6547State(this); //the robot

        /*
        Make the lift stay where it's at.  The scissor lift's force is stronger
        then a motor on brake mode, so we have the lift motor try to keep is's original encoder value
        by going up or down if the scissor lift pushes the motor off its original
        target value
         */

        //bot.setRunLift(true);

        bot.setLiftTargetPos(0);

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
        if (isRed) bot.setPoseEstimate(new Pose2d(-36, -62,Math.toRadians(faceForwardDeg)));
        else bot.setPoseEstimate(new Pose2d(-36,62,Math.toRadians(faceForwardDeg)));

        telemetry.addData("Ready to start","");
        telemetry.update();
        telemetry.log().add("Ready to start");
        RobotLog.v("Waiting For Start...");
        while (!isStarted() && !isStopRequested())
        {
            bot.setLiftToTargetPos(0,50);
            telemetry.addData("Lift Pos",bot.lift.getCurrentPosition());
            telemetry.update();
        }
        telemetry.log().add("Starting");
        RobotLog.v("Starting");

        //Go to Skystones
        bot.followTrajectorySync(bot.trajectoryBuilder()
        .splineTo(new Pose2d(-35,-31*yModifer,Math.toRadians(faceForwardDeg)))
        .build());

        RobotLog.v("Drove to Stones");

        sleep(500);

        bot.setRunIntakeUntilStone(true);

        if (bot.isSkystone(bot.colorSensorSideLeft))
        {
            // ---SKYSTONE LEFT---
            telemetry.log().add("LEFT");
            bot.skyStoneLoc = SkyStoneLoc.LEFT;

            if (isRed) { //if red
                bot.followTrajectorySync(bot.trajectoryBuilder()
                        .back(4)
                        .addTemporalMarker(0,new IntakeUntilStone(bot))
                        .lineTo(new Vector2d(-52, -32))
                        .build());
            }
            else //if blue (this is the same path as going right for RED)
            {
                bot.followTrajectorySync(bot.trajectoryBuilder()
                        .back(4)
                        .addTemporalMarker(0,new IntakeUntilStone(bot))
                        .lineTo(new Vector2d(-18,32))
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
                        .addTemporalMarker(0,new IntakeUntilStone(bot))
                        .lineTo(new Vector2d(-18, -32))
                        .build());
            }
            else //if blue (path is same as left for RED)
            {
                bot.followTrajectorySync(bot.trajectoryBuilder()
                        .back(4)
                        .addTemporalMarker(0,new IntakeUntilStone(bot))
                        .lineTo(new Vector2d(-52, 32))
                        .build());
            }
        }
        else //center (same for both colors)
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .lineTo(new Vector2d(-34,-20))
            .build());
            //---SKYSTONE CENTER---
            //go back a bit
            telemetry.log().add("CENTER");
            bot.skyStoneLoc = SkyStoneLoc.CENTER;

        }

        bot.getRuntime().reset();
        while (bot.getRuntime().seconds() < .5 && !bot.isStone(bot.intakeColorSensor) && opModeIsActive())
        {
            bot.runAtAllTimes();
        }
        if (!bot.isStone(bot.intakeColorSensor) && bot.getRunIntakeUntilStone()) //if running intake
        {
            RobotLog.v("Droving forward to get stone");
            bot.followTrajectorySync(bot.trajectoryBuilder()
            .forward(6)
            .build());
        }
        //open intake
        //spline to under the skybridge to foundation
        //bot.setPoseEstimate(new Pose2d(bot.getPoseEstimate().getX(),bot.getPoseEstimate().getY(),norm(bot.getPoseEstimate().getHeading()-Math.toRadians(180))));
        if (isRed) bot.turnRealtiveSync(Math.toRadians(300));
        bot.followTrajectorySync(bot.trajectoryBuilder(false)
              //  .reverse()
                .splineTo(new Pose2d(-5,-44*yModifer,Math.toRadians(0)))
                .splineTo(new Pose2d(23,-45*yModifer,Math.toRadians(0)))
                //.splineTo(new Pose2d(50,-24*yModifer,Math.toRadians(faceForwardDeg)))
                .build());
        for (int i=0; i < 2; i++) {
            bot.turnRealtiveSync(Math.toRadians(faceBackwardDeg));
        }

        bot.followTrajectorySync(bot.trajectoryBuilder()
        .back(6).build());
        bot.setFondationGrabber(1);

        sleep(500);

        bot.grabStoneInIntake();

        sleep(500);

        //wait for foundation grabber to grab and grab the block

        bot.stopIntake();

        bot.moveLift(3000,50);

        RobotLog.d("pulling foundation");

        bot.ExtendGrabberSlide();
        //pull foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .forward(36)
                //.splineTo(new Pose2d(33, -60*yModifer, Math.toRadians(180)))
                .build());

        bot.openGrabber();
        bot.RetractGrabberSlide();

        //turn founation toward wall

        if (isRed) bot.turnSync(Math.toRadians(-300));
        else bot.turnSync(Math.toRadians(300));
        //go to other stone

        bot.setFondationGrabber(0);
        bot.setRunLift(true);
        bot.setLiftTargetPos(0);
        //bot.moveLift(0,50);
        bot.turnRealtiveSync(Math.toRadians(180));

        RobotLog.d("Placing Foundation");
        //go to other stone
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(24)
                .addTemporalMarker(1,new Intake(bot,1))
                //.strafeRight(8)
                .build());

        RobotLog.d("Driving to under Skybridge");
        bot.intake(1);
        bot.followTrajectory(bot.trajectoryBuilder()
                .addTemporalMarker(2,new StopIntake(bot))
        .splineTo(new Pose2d(0,-39*yModifer,Math.toRadians(180)))
        .build());

        bot.setRunIntakeUntilStone(true);

        double constX= -25;
        double constY = -38;

        double degToTurn;

        RobotLog.d("grabbing second SkyStone");

        //prepare to grab the other Skystone
        if ((bot.skyStoneLoc == SkyStoneLoc.RIGHT && isRed) || (bot.skyStoneLoc == SkyStoneLoc.LEFT && !isRed))
        {
            if (isRed) degToTurn = 135;
            else degToTurn = 225;
            telemetry.log().add("GOING RIGHT");
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(constX+10,constY*yModifer,Math.toRadians(180)))
            .splineTo(new Pose2d(-40,-22*yModifer,Math.toRadians(degToTurn)))
            .build());
        }
        else if (bot.skyStoneLoc == SkyStoneLoc.CENTER)
        {
            if (isRed) degToTurn = 130;
            else  degToTurn = 230;

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(constX-5,constY*yModifer,Math.toRadians(180)))
            .splineTo(new Pose2d(-45,-22*yModifer, Math.toRadians(degToTurn)))
            .build());


        }
        else if ((bot.skyStoneLoc == SkyStoneLoc.LEFT && isRed) || (bot.skyStoneLoc == SkyStoneLoc.RIGHT && !isRed))
        {
            if (isRed) degToTurn = 150;
            else degToTurn = 210;

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(constX,(constY*yModifer)-2,Math.toRadians(180)))
                    .splineTo(new Pose2d(-57,-20*yModifer,Math.toRadians(degToTurn)))
                    .build());
        }

        //if the bot has not grabbed a stone yet, go forward a bit
        if (!bot.isStone(bot.intakeColorSensor)) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .forward(15)
                    .build());
        }

        RobotLog.d("Delelivering Stone");

        //deliver second stone
       bot.followTrajectorySync(bot.trajectoryBuilder()
       .splineTo(new Pose2d(0,-35*yModifer,Math.toRadians(180)))
               .addTemporalMarker(1.5, new Intake(bot,1))
               .splineTo(new Pose2d(-8,-35*yModifer,Math.toRadians(180)))
       .build());

        //park under skybridge

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeTo(new Vector2d(0,-32*yModifer))
       .build());

       //strafe to give more room to allience partner

       bot.followTrajectorySync(bot.trajectoryBuilder()
       .strafeRight(15)
       .build());

       bot.stopIntake();

       bot.saveRobotPos();
        //save gyro angle
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME,Math.toDegrees(bot.getPoseEstimate().getHeading()) - 90);

        //run until auton is over to keep the scissor lift down
        while (opModeIsActive()) {
            bot.outputTelemetry();
        }

    }
    double norm(double x)
    {
        while (x>=360) x-=360;
        while (x<=0) x+=360;
        return x;
    }
}
