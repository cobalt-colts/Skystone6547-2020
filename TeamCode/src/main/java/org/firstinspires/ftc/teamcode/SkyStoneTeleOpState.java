package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

import edu.spa.ftclib.internal.state.ToggleBoolean;
import edu.spa.ftclib.internal.state.ToggleDouble;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "SkyStone Tele-op State", group = "_teleOp")
public class SkyStoneTeleOpState extends LinearOpMode {

    public static double slideSpeed = .004; //speed of horizontal slide in servo position units

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
    private ToggleDouble grabberToggle = new ToggleDouble(new double[] {0, .4}, 0);

    private DriveTrain6547 bot; //the robot class

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); //makes telemetry output to the FTC Dashboard
        bot = new DriveTrain6547(this);
        telemetry.update();

        bot.disableEncoders();

        telemetry.log().add("DONE INITIALING");

        bot.setPoseEstimate(new Pose2d(-36,-63,Math.toRadians(90)));

        //get the angle the robot was at when auton ended
        bot.setAngleZzeroValue(-bot.readFile(bot.GYRO_ANGLE_FILE_NAME));
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, 0); //reset the old angle to zero

        telemetry.log().add("Ready to start");
        telemetry.log().add("gyro angle: " + bot.getIMUAngle());
        telemetry.log().add("lift max: " + bot.liftMax);

        bot.setBulkReadAuto();

        waitForStart();

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
            if (bot.x1.onPress()) speedModifer=.30;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifer=.70;
            if (bot.a1.onPress() && !bot.start1.isPressed()) speedModifer=1.0;

            if (bot.y1.onPress()) feildRealtive.toggle(); //toggle field realtive

            if (feildRealtive.output()) //field realtive code
            {
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //get angle
                double robotAngle = Math.toRadians(bot.getIMUAngle()); //angle of robot
                double rightX=gamepad1.right_stick_x; //rotation
                rightX*=.5; //half rotation value for better turning
                //offset the angle by the angle of the robot to make it field realtive
                leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;
                rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
                leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
                rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;
            }
            else //regular drive (different math because this is faster than sins and cosines0
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
            if (bot.rightTrigger2.onPress()) //intake
            {
                if (!intake)
                {
                    intake = true;
                    outtake = false;
                    bot.intake(1);
                }
                else //intake button pressed again
                {
                    intake = false;
                    outtake = false;
                    bot.stopIntake();
                }
            }
            else if (bot.leftTrigger2.onPress()) //outtake
            {
                if (!outtake) {
                    intake = false;
                    outtake = true;
                    bot.outtake(1);
                }
                else //outtake button pressed again
                {
                    intake = false;
                    outtake = false;
                    bot.stopIntake();
                }
            }

            if (bot.a2.onPress()) //toggle fondation grabber
            {
                fondationGrabberPos.toggle();
                bot.setFondationGrabber(fondationGrabberPos.output());
            }

            if (bot.b2.onPress()) //toggle stone grabber
            {
                grabberToggle.toggle();
                bot.setGrabber(grabberToggle.output());
            }

            double liftSpeed = -gamepad2.left_stick_y;

            if (bot.lift.getCurrentPosition() > bot.getLiftStartingPos() + 1000)
            {
                //probably should change this if statement, but I'm too scared too.
            }
            else
            {
                if (liftSpeed > 0) liftSpeed*=.75;
            }
            /*
            Lift controls
            Deadzone of .05
            has a maximum, but no minimum
             */
            if (liftSpeed > .05 && bot.lift.getCurrentPosition() < bot.liftMax) //if lift is below max and speed is outside of deadzone
            {
                bot.setLiftPower(liftSpeed);
            }
            else if (liftSpeed < -.05) //if speed is down and outside of deadzone
            {
                bot.setLiftPower(liftSpeed);
            }
            else
            {
                bot.setLiftPower(0);
            }

            if (bot.rightBumper2.isPressed()) bot.updateServo(bot.grabberSlide, 1, slideSpeed, bot.grabberMax, bot.grabberMin); //move horizontal slide back
            if (bot.leftBumper2.isPressed()) bot.updateServo(bot.grabberSlide, -1, slideSpeed, bot.grabberMax, bot.grabberMin); //move horizontal slide forward

            if (gamepad1.right_bumper && gamepad1.left_bumper) //calibrate gyro
            {
                bot.setAngleZzeroValue(-Math.toDegrees(bot.getRawExternalHeading()));
            }

            /*
            Telemetry
             */
            //Pose2d pos = bot.getPoseEstimate();
            //bot.setPoseEstimate(new Pose2d(pos.getX(), pos.getY(), bot.getRawExternalHeading()+Math.toRadians(90)));
            bot.updateRobotPosRoadRunner(); //display robot's position
        }
        bot.stopRobot();
    }
}
