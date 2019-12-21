package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

import edu.spa.ftclib.internal.state.ToggleBoolean;
import edu.spa.ftclib.internal.state.ToggleDouble;

/*
This is the tele-op we use to drive the robot
 */
@Config
@TeleOp(name = "SkyStone Tele-op Qualifier")
public class SkyStoneTeleOpQualifer extends LinearOpMode {

    public static double FoundationGrabberMin=0;
    public static double FoundationGrabberMax = .9;
    public static double FrontGrabberMin=0;
    public static double FrontGrabberMax=1;
    public static double BackGrabberMin=0;
    public static double BackGrabberMax=1;

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
    ToggleDouble frontGrabberPos = new ToggleDouble(new double[] {0,1});
    ToggleDouble backGrabberPos = new ToggleDouble(new double[]{0,1});

    double liftPhoneHeight = 5552;

    SkyStone6547Qualifter bot;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        bot = new SkyStone6547Qualifter(this);
        telemetry.update();

        bot.zeroEncoders();

        double oldGyroAngle=bot.readFile(bot.GYRO_ANGLE_FILE_NAME); //get the angle the robot was at when auton ended
        bot.angleZzeroValue=oldGyroAngle;
        bot.writeFile(bot.GYRO_ANGLE_FILE_NAME, 0); //reset the old angle to zero

        telemetry.log().add("Ready to start");
        telemetry.log().add("gyro angle: " + bot.getIMUAngle());

        waitForStart();

        while (opModeIsActive()) {

//            fondationGrabberPos.changeValue(FoundationGrabberMax,1);
//            fondationGrabberPos.changeValue(FoundationGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMin,0);
//            frontGrabberPos.changeValue(FrontGrabberMax,1);
//            backGrabberPos.changeValue(BackGrabberMin,0);
//            backGrabberPos.changeValue(BackGrabberMax,1);

            bot.updateGamepads();

            if (bot.y1.onPress()) speedModifer=.3;
            if (bot.b1.onPress() && !bot.start1.isPressed()) speedModifer=.7;
            if (bot.a1.onPress() && !bot.start1.isPressed()) speedModifer=1;

            if (bot.x1.onPress()) feildRealtive.toggle(); //toggle field realtive

            if (feildRealtive.output()) //field realtive code
            {
                double speed = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                double LeftStickAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double robotAngle = Math.toRadians(bot.getIMUAngle());
                double rightX=gamepad1.right_stick_x;
                rightX*=.5;
                leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;
                rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
                leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
                rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;
            }
            else //regular drive
            {
                leftFrontPower=gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                rightFrontPower=gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                leftBackPower=gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
                rightBackPower=gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            }

            bot.LeftFront.setPower(leftFrontPower*speedModifer); //set the power according to the speed modifer
            bot.RightFront.setPower(rightFrontPower*speedModifer);
            bot.LeftBack.setPower(leftBackPower*speedModifer);
            bot.RightBack.setPower(rightBackPower*speedModifer);
            
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
            if (bot.leftBumper2.onPress())
            {
                frontGrabberPos.toggle();
                bot.setFrontGrabber(frontGrabberPos.output());
            }
            if (bot.rightBumper2.onPress())
            {
                backGrabberPos.toggle();
                bot.setBackGrabber(backGrabberPos.output());
            }
            if (bot.y2.onPress())
            {
                bot.grabBlock();
            }

            double liftPower = -gamepad2.left_stick_y;
            double liftPos = Math.abs(bot.lift.getCurrentPosition());

            if (liftPower > 0 && bot.lift.getCurrentPosition()<bot.liftMax) //going up
            {
                bot.lift.setPower(liftPower);
            }
            else if (liftPower < 0 && bot.lift.getCurrentPosition()>bot.liftMin) //going down
            {
                bot.lift.setPower(liftPower);
            }
            bot.lift.setPower(-gamepad2.left_stick_y); //move lift

            if (gamepad1.right_bumper && gamepad1.left_bumper) //calibrate gyro
            {
                bot.angleZzeroValue = -bot.angles.firstAngle;
            }

            if (bot.getRobotPositionX() < 2)
            {
                bot.scream();
                telemetry.log().add("Screaming");
            }
            else if (bot.getRobotPositionY() < 2)
            {
                bot.scream();
                telemetry.log().add("Screaming");
            }

           telemetry.addData("Back grabber pos", bot.backGrabber.getPosition());
            telemetry.addData("Front grabber pos", bot.frontGrabber.getPosition());
            telemetry.addData("IMU angle", bot.getIMUAngle());
            telemetry.addData("zero value" , bot.angleZzeroValue);
            telemetry.addData("angles.firstAngle", bot.angles.firstAngle);
            // telemetry.addData("Left Front current Pos", LeftFront.getCurrentPosition());
            telemetry.addData("Left Back current Pos", bot.LeftBack.getCurrentPosition());
            // telemetry.addData("Right Front current Pos", RightFront.getCurrentPosition());
            // telemetry.addData("Right Back current Pos", RightBack.getCurrentPosition());
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
