package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathBuilderException;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RoadRunner.actions.Intake;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.IntakeUntilStone;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.MoveGrabberSlideTime;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.Outtake;
import org.firstinspires.ftc.teamcode.RoadRunner.actions.StopIntake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547State;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;

/**
 * Created by Drew from 6547 on 9/27/2019.
 */
@Autonomous(name = "GC RED/Blue double skystone Foundation State", group = "auton")
public class DoubleSkystoneFoundationGC extends LinearOpMode {

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
        if (isRed) bot.setPoseEstimate(new Pose2d(-36, -62,Math.toRadians(faceForwardDeg)));
        else bot.setPoseEstimate(new Pose2d(-36,62,Math.toRadians(faceForwardDeg)));

        telemetry.addData("Ready to start","");
        telemetry.update();
        telemetry.log().add("Ready to start");
        RobotLog.v("Waiting For Start...");



        while (!isStopRequested() && !isStarted() && !bot.isTouchSensorPressed())
        {
            bot.setLiftPower(-1);
        }
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

        //Go to Skystones
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .splineTo(new Pose2d(-35,-34*yModifer,Math.toRadians(faceForwardDeg)))
                .build());

        RobotLog.v("Drove to Stones");

        //sleep(500);

        bot.setRunIntakeUntilStone(true);

        if (bot.isSkystone(bot.colorSensorSideLeft))
        {
            // ---SKYSTONE LEFT---
            telemetry.log().add("LEFT");
            bot.skyStoneLoc = SkyStoneLoc.LEFT;

            bot.driveBackward(4);
            bot.strafeLeft(8);
            if (!bot.isStoneAtIntake()) bot.driveForward(15);
            //bot.followTrajectorySync(bot.trajectoryBuilder()
              //      .lineToLinearHeading(new Vector2d(-40,-36),Math.toRadians(0))
                //    .build());

//            if (isRed) { //if red
//                bot.followTrajectorySync(bot.trajectoryBuilder()
//                        //.back(4)
//                        .addTemporalMarker(0,new IntakeUntilStone(bot))
//                        .lineTo(new Vector2d(-52, -32))
//                        .build());
//            }
//            else //if blue (this is the same path as going right for RED)
//            {
//                bot.followTrajectorySync(bot.trajectoryBuilder()
//                        .back(4)
//                        .addTemporalMarker(0,new IntakeUntilStone(bot))
//                        .lineTo(new Vector2d(-18,32))
//                        .build());
//            }

        }
        else if (bot.isSkystone(bot.colorSensorSideRight))
        {
            //---SKYSTONE RIGHT---
            telemetry.log().add("RIGHT");
            bot.skyStoneLoc = SkyStoneLoc.RIGHT;

            bot.driveBackward(4);
            bot.strafeRight(12);
            if (!bot.isStoneAtIntake()) bot.driveForward(12);
//            if (isRed) {
                //drive back a bit and strafe right
//                bot.followTrajectorySync(bot.trajectoryBuilder()
//                        //.back(4)
//                        //.addTemporalMarker(0,new IntakeUntilStone(bot))
//                        .lineToLinearHeading(new Vector2d(-28, -21),Math.toRadians(45))
//                        .build());
//
//                bot.followTrajectorySync(bot.trajectoryBuilder()
//                .lineToLinearHeading(new Vector2d(-10,-36),Math.toRadians(0))
//                    .build());
//            }
//            else //if blue (path is same as left for RED)
//            {
//                bot.followTrajectorySync(bot.trajectoryBuilder()
//                        .back(4)
//                        .addTemporalMarker(0,new IntakeUntilStone(bot))
//                        .lineTo(new Vector2d(-52, 32))
//                        .build());
//            }
        }
        else //center (same for both colors)
        {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .lineTo(new Vector2d(-34,-22*yModifer))
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
        // It seems to get lost during this, causing future trajectories to fail and drive to opponent's side sometimes.

//        if (!bot.isStone(bot.intakeColorSensor) && bot.getRunIntakeUntilStone()) //if running intake
//        {
//            RobotLog.v("Driving forward to get stone");
//            bot.followTrajectorySync(bot.trajectoryBuilder()
//                    .forward(6)
//                    .build());
//        }

        //spline to under the skybridge to foundation
        //bot.setPoseEstimate(new Pose2d(bot.getPoseEstimate().getX(),bot.getPoseEstimate().getY(),norm(bot.getPoseEstimate().getHeading()-Math.toRadians(180))));
//        bot.followTrajectorySync(bot.trajectoryBuilder(false)
//                .back(12)
//                .build());

        if (isRed) {

            //bot.turnRealtiveSync(Math.toRadians(30));
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(24)
                    .build());
            TrajectoryBuilder builder = new TrajectoryBuilder(bot.getPoseEstimate(), bot.getPoseEstimate().getHeading(),
                    new DriveConstraints(
                            40.0, 10.0, 0.0,
                            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
                    ));
            // Let's let trajectories handle this instead
//        if (isRed) bot.turnRealtiveSync(Math.toRadians(300));
            bot.followTrajectorySync(builder
                    .splineTo(new Pose2d(-5, -38, Math.toRadians(0)))
                    .build());

            // Added:
            bot.followTrajectorySync(bot.trajectoryBuilder(false)
                    .splineTo(new Pose2d(35, -47, Math.toRadians(0)))
                    .build());
        } else {
           // bot.turnRealtiveSync(Math.toRadians(30));

            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .back(24)
                    .build());
           // bot.turnRealtiveSync(Math.toRadians(30));

            // Let's let trajectories handle this instead
//        if (isRed) bot.turnRealtiveSync(Math.toRadians(300));
            bot.followTrajectorySync(bot.trajectoryBuilder(false)
                    .splineTo(new Pose2d(-5, 35, Math.toRadians(0)))
                    .build());

            // Added:
            bot.followTrajectorySync(bot.trajectoryBuilder(false)
                    .splineTo(new Pose2d(54, 31, Math.toRadians(0)))
                    .build());
        }

        // Turn to face foundation
        for (int i=0; i < 3; i++) {
            bot.turnRealtiveSync(Math.toRadians(faceBackwardDeg));
        }

        bot.openGrabber();

        bot.intake(.6);         // Start intaking slowly to move the stone against the foundation

        // Back up into the foundation
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(12)
                .build());
        bot.setFondationGrabber(1);     // Grab foundation

        bot.RetractGrabberSlide();  // In case it's come out some

        bot.setLiftPower(-.25);   // Make sure it's down
        sleep(250);

        bot.grabStoneInIntake();
        sleep(250);

        //bot.setLiftPower(0);


        //wait for foundation grabber to grab and grab the block


        //bot.moveLift(800,50);
        //sleep(500);
        bot.setLiftTargetPos(800);

        bot.stopIntake();

        RobotLog.d("pulling foundation");

        //bot.ExtendGrabberSlide();
       // bot.moveGrabberSlideForTime(1,1000);
        //sleep(400);

        //pull foundation
        if (isRed) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .addTemporalMarker(1, new MoveGrabberSlideTime(bot, 1, 1000))
                    .forward(44)
                    //.splineTo(new Pose2d(33, -60*yModifer, Math.toRadians(180)))
                    .build());
        }
        else { //if blue
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .addTemporalMarker(1, new MoveGrabberSlideTime(bot, 1, 1000))
                    .forward(36)
                    //.splineTo(new Pose2d(33, -60*yModifer, Math.toRadians(180)))
                    .build());
        }

        bot.stopGrabberSlide();


        bot.openGrabber();
        //sleep(500);

        //bot.RetractGrabberSlide();
        bot.moveGrabberSlideForTime(-1,1000);

        //turn foundation toward wall

        if (isRed) bot.turnSync(Math.toRadians(-300));
        else bot.turnSync(Math.toRadians(220));

        bot.stopGrabberSlide();

        //go to other stone
        bot.setFondationGrabber(0);
        bot.closeGrabber();
        bot.setRunLift(true);
        bot.setLiftTargetPos(0);

        //bot.moveLift(0,50);
        //bot.turnRealtiveSync(Math.toRadians(180));

        bot.openGrabber();

        RobotLog.d("Placing Foundation");
        bot.driveBackward(6);
        // No need to push it against the wall this cycle
//        bot.followTrajectorySync(bot.trajectoryBuilder()
//                .back(24)
//                .addTemporalMarker(1,new Intake(bot,1))
//                //.strafeRight(8)
//                .build());


        RobotLog.d("Driving to under Skybridge");
        bot.intake(1);
        if (isRed) {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    //.addTemporalMarker(2,new StopIntake(bot))
                    .splineTo(new Pose2d(-5, -30, Math.toRadians(180)))
                    .build());
        }
        else {
            bot.followTrajectorySync(bot.trajectoryBuilder()
                    //.addTemporalMarker(2,new StopIntake(bot))
                    .splineTo(new Pose2d(-5, 39, Math.toRadians(180)))
                    .build());
        }
        bot.stopIntake();
        RobotLog.d("Driving to under Skybridge DONE");
        if (isRed) bot.strafeRight(12);
        else bot.strafeLeft(12);

        while(opModeIsActive()); // stop here



        //------------------
        //-------------------
        //------------------

        bot.setRunIntakeUntilStone(true);

        double constX= -25;
        double constY = -36;

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
                    .build());


            bot.followTrajectorySync(bot.trajectoryBuilder()
                    .splineTo(new Pose2d(-58,-22*yModifer, Math.toRadians(degToTurn)))
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

        bot.turnRealtiveSync(Math.toRadians(210));

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .addTemporalMarker(1.5, new Outtake(bot, 1))
                .splineTo(new Pose2d(-10, -40 * yModifer, Math.toRadians(0)))
                .splineTo(new Pose2d(10,-38 * yModifer,Math.toRadians(0)))
                .build());

        //bot.turnRealtiveSync(180);
        //bot.intake(1);
        bot.followTrajectorySync(bot.trajectoryBuilder()
                .back(12)
                .build());


        //park under skybridge

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .strafeTo(new Vector2d(0,-32*yModifer))
                .build());

        //strafe to give more room to alliance partner

        bot.followTrajectorySync(bot.trajectoryBuilder()
                .strafeRight(15)
                .build());


        ///////////////////////////////////////////////////

        ///////////////////////////////////////////////////

        bot.setFondationGrabber(0);     // Un-grab foundation

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
