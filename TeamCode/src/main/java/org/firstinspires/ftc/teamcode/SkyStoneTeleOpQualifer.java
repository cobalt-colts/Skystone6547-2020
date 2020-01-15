package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;
import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

import edu.spa.ftclib.internal.state.ToggleBoolean;
import edu.spa.ftclib.internal.state.ToggleDouble;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "SkyStone Tele-op Qualifier")
public class SkyStoneTeleOpQualifer extends LinearOpMode {

    public static double grabberMin = 0;
    public static double grabberMax = 1;
    public static double slideSpeed = .004;

    public static double speedModifer=.7; //lowers the speed so it's easier to drive

    boolean intake = false;
    boolean outtake = false;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    ToggleBoolean feildRealtive = new ToggleBoolean(true);

    //edit the array to change the foundation grabber position(s)
    ToggleDouble fondationGrabberPos = new ToggleDouble(new double[] {0,0.9},0);
    ToggleDouble grabberToggle = new ToggleDouble(new double[] {grabberMin, grabberMax}, 0);

    DriveTrain6547 bot;

    @Override
    public void runOpMode() {

        bot = new DriveTrain6547(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        telemetry.update();

        bot.zeroEncoders();

        double oldGyroAngle=bot.readFile(bot.GYRO_ANGLE_FILE_NAME); //get the angle the robot was at when auton ended
        bot.angleZzeroValue=oldGyroAngle;
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, 0); //reset the old angle to zero

        telemetry.log().add("Ready to start");
        telemetry.log().add("gyro angle: " + bot.getIMUAngle());

        waitForStart();

        while (opModeIsActive()) {

            grabberToggle = new ToggleDouble(new double[] {grabberMin, grabberMax}, grabberToggle.getToggleIndex());
//            fondationGrabberPos.changeValue(FoundationGrabberMax,1);
//            fondationGrabberPos.changeValue(FoundationGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMax,1);
//            backGrabberPos.changeValue(BackGrabberMin,0);
//            backGrabberPos.changeValue(BackGrabberMax,1);

            bot.updateGamepads();

            if (bot.y1.onPress()) speedModifer=.30;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifer=.70;
            if (bot.a1.onPress() && !bot.start1.isPressed()) speedModifer=1.0;

            if (bot.x1.onPress()) feildRealtive.toggle(); //toggle field realtive

            if (feildRealtive.output()) //field realtive code
            {
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double robotAngle = Math.toRadians(bot.getIMUAngle());
                double rightX=gamepad1.right_stick_x;
                rightX*=.5;
                leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;
                rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
                leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
                rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;
            }
            else //regular drive
            {
                leftFrontPower=-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightFrontPower=-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                leftBackPower=-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightBackPower=-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            }

            bot.setMotorPowers(leftFrontPower*speedModifer, leftBackPower*speedModifer, rightBackPower*speedModifer, rightFrontPower*speedModifer);
            
            if (bot.rightTrigger2.onPress())
            {
                if (!intake)
                {
                    intake = true;
                    outtake = false;
                    bot.intake(1);
                }
                else
                {
                    intake = false;
                    outtake = false;
                    bot.stopIntake();
                }
            }
            else if (bot.leftTrigger2.onPress())
            {
                if (!outtake) {
                    intake = false;
                    outtake = true;
                    bot.outtake(1);
                }
                else
                {
                    intake = false;
                    outtake = false;
                    bot.stopIntake();
                }
            }

            if (bot.a2.onPress())
            {
                fondationGrabberPos.toggle();
                bot.setFondationGrabber(fondationGrabberPos.output());
            }

            if (bot.b2.onPress())
            {
                grabberToggle.toggle();
                bot.setGrabber(grabberToggle.output());
            }

//            double liftPower = gamepad2.left_stick_y;
//            double liftPos = Math.abs(bot.lift.getCurrentPosition());
//
//            if (liftPower > 0 && bot.lift.getCurrentPosition()<bot.liftMax) //going up
//            {
//                bot.lift.setPower(liftPower);
//            }
//            else if (liftPower < 0 && bot.lift.getCurrentPosition()>bot.liftMin) //going down
//            {
//                bot.lift.setPower(liftPower);
//            }
            //bot.setLiftPower(gamepad2.left_stick_y); //move lift

            bot.lift.setPower(gamepad2.right_stick_y*.5);
            bot.liftLeft.setPower(gamepad2.left_stick_y*.5);

            if (bot.rightBumper2.isPressed()) bot.updateServo(bot.grabberSlide, 1, slideSpeed, .95, .80);
            if (bot.leftBumper2.isPressed()) bot.updateServo(bot.grabberSlide, -1, slideSpeed, .95, .80);

            bot.updateServo(bot.grabberSlide, gamepad2.right_stick_x, slideSpeed, 1, .80);

            if (gamepad1.right_bumper && gamepad1.left_bumper) //calibrate gyro
            {
                bot.angleZzeroValue = -Math.toDegrees(bot.getRawExternalHeading());
            }

            telemetry.addData("IMU angle", bot.getIMUAngle());
            telemetry.addData("zero value" , bot.angleZzeroValue);
            telemetry.addData("right lift pos", bot.lift.getCurrentPosition());
            telemetry.addData("left lift pos", bot.liftLeft.getCurrentPosition());
            telemetry.addData("angles.firstAngle",Math.toDegrees(bot.getRawExternalHeading()));
            telemetry.addData("Grabber POS", bot.grabber.getPortNumber());
            telemetry.addData("Grabber Slider POS",bot.grabberSlide.getPosition());
            telemetry.addData("Left Back current Pos", bot.leftRear.getCurrentPosition());
            telemetry.addData("Left Front current Pos", bot.leftFront.getCurrentPosition());
            telemetry.addData("Right Front current Pos", bot.rightFront.getCurrentPosition());
            telemetry.addData("Right Back current Pos", bot.rightRear.getCurrentPosition());
            telemetry.addData("A is full speed, B is half speed, Y is quarter speed","");
            telemetry.addData("Field Realitive Driving ", feildRealtive.output());
            // telemetry.addData("Left Trigger", gamepad1.left_trigger);
            // telemetry.addData("Right Trigger", gamepad1.right_trigger);
            // telemetry.addData("Left Front Power",leftFrontPower);
            // telemetry.addData("Right Front Power",rightFrontPower);
            // telemetry.addData("Left Back Power",leftBackPower);
            // telemetry.addData("Right Back Power",rightBackPower);
            telemetry.addData("Speed modifer", speedModifer);
            telemetry.update();
        }
        bot.stopRobot();
    }
}
