package org.firstinspires.ftc.teamcode.realsense;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveTrain6547Offseason;
import org.firstinspires.ftc.teamcode.roadRunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.state.ToggleBoolean;
import org.firstinspires.ftc.teamcode.util.state.ToggleDouble;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "SkyStone Tele-op Road Runner RealSense", group = "_teleOp")
public class RoadRunnerRealSenseTest extends LinearOpMode {

    public static double slideSpeed = .0045; //speed of horizontal slide in servo position units

    public static double speedModifer=.7; //lowers the speed so it's easier to drive

    private boolean intake = false;
    private boolean outtake = false;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private ToggleBoolean feildRealtive = new ToggleBoolean(true);

    //edit the array to change the foundation grabber position(s)
    private ToggleDouble fondationGrabberPos = new ToggleDouble(new double[] {0,1},0);
    private ToggleDouble grabberToggle = new ToggleDouble(new double[] {0, 1}, 0);

    private DriveTrain6547Realsense bot; //the robot class

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547Realsense(this);
        telemetry.update();

        bot.setPoseEstimate(new Pose2d());

        bot.disableEncoders();

        telemetry.log().add("DONE INITIALING");

//        double startingAngle = bot.readFile(bot.GYRO_ANGLE_FILE_NAME);
//
//        bot.setAngleZzeroValue(-startingAngle);
        //bot.setPoseEstimate(new Pose2d(-36,-63,startingAngle));

        //get the angle the robot was at when auton ended
        //bot.setAngleZzeroValue(-bot.readFile(bot.GYRO_ANGLE_FILE_NAME));
      //  bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, 0); //reset the old angle to zero

        telemetry.log().add("Ready to start");
        telemetry.log().add("gyro angle: " + bot.getIMUAngle());

        bot.setBulkReadAuto();

        waitForStart();

        bot.startRealsense();

       // bot.setPoseEstimate(new Pose2d(0,0,0));

        while (opModeIsActive()) {

            //grabberToggle = new ToggleDouble(new double[] {bot.grabberMin, bot.grabberMax}, grabberToggle.getToggleIndex());
//            fondationGrabberPos.changeValue(FoundationGrabberMax,1);
//            fondationGrabberPos.changeValue(FoundationGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMax,1);
//            backGrabberPos.changeValue(BackGrabberMin,0);
//            backGrabberPos.changeValue(BackGrabberMax,1);

            bot.updateGamepads();

            /*
            Speed Modifers
             */
            if (bot.x1.onPress()) speedModifer=.60;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifer=.9;
            if (bot.a1.onPress() && !bot.start1.isPressed()) speedModifer=1.7; //trig math caps speed at .7, 1.3 balences it out

            if (bot.y1.onPress()) feildRealtive.toggle(); //toggle field realtive

            if (feildRealtive.output()) //if field relative is enabled
            {
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //get angle
                double robotAngle = Math.toRadians(bot.getIMUAngle()); //angle of robot
                double rightX=gamepad1.right_stick_x*2; //rotation
                rightX*=.5; //half rotation value for better turning
                //offset the angle by the angle of the robot to make it field realtive
                leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;
                rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
                leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
                rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;

                telemetry.addData("LS angle",Math.toDegrees(LeftStickAngle));
                telemetry.addData("driving toward",LeftStickAngle-robotAngle);
                telemetry.addData("ROBOT ANGLE",Math.toDegrees(robotAngle));
                telemetry.addData("RAW ANGLE", Math.toDegrees(bot.getRawExternalHeading()));
            }
            else //regular drive (different math because this is faster than sins and cosines
            {
                leftFrontPower=-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightFrontPower=-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                leftBackPower=-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightBackPower=-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            }

            //set motor powers based on previous calculations
            bot.setMotorPowers(leftFrontPower*speedModifer, leftBackPower*speedModifer, rightBackPower*speedModifer, rightFrontPower*speedModifer);

            /*
            Toggle Intake:
            Triggers control intake/outtake
            Push the same trigger twice to turn the intake motors off
             */

            //old way to move servo
            //if (bot.rightBumper2.isPressed()) bot.updateServo(bot.grabberSlide, 1, slideSpeed, bot.grabberMax, bot.grabberMin); //move horizontal slide back
            //if (bot.leftBumper2.isPressed()) bot.updateServo(bot.grabberSlide, -1, slideSpeed, bot.grabberMax, bot.grabberMin); //move horizontal slide forward
            if (gamepad1.right_bumper && gamepad1.left_bumper) //calibrate gyro
            {
                double zeroVal = -Math.toDegrees(bot.getRawExternalHeading());
                bot.setAngleZzeroValue(zeroVal);
                telemetry.log().add("Calibrated, set zero value to" + zeroVal);
            }

            final int robotRadius = 9; // inches
            /*
            Telemetry
             */
            //Pose2d pos = bot.getPoseEstimate();
            //bot.setPoseEstimate(new Pose2d(pos.getX(), pos.getY(), bot.getRawExternalHeading()+Math.toRadians(90)));
            bot.updateRobotPosRoadRunner(); //display robot's position
        }
        bot.stopRealsense();
        bot.stopRobot();
    }
}
