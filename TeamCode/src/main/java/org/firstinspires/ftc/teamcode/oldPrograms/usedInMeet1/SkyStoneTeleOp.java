package org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1.SkyStone6547;

/*
This is the tele-op we use to drive the robot
 */
@TeleOp(name = "SkyStone Tele-op")
@Disabled
public class SkyStoneTeleOp extends SkyStone6547 {

    double speedModifer=.7; //lowers the speed so it's easier to drive

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    boolean feildRealtive=true;

    boolean isGamepadPressed=false;//used to program toggle function in the X button in gamepad 1

    boolean isXPressed=false; //used to program toggle function in the X button in gamepad 2

    boolean isTriggerPressed=false; //used to program toggle function in the triggers

    boolean isAPressed = false;

    boolean foundationGrabberDown = false;
    double targetSpinnerPos=0; //the position of the spinner
    @Override
    public void runOpMode() {
        INIT(hardwareMap);
        telemetry.update();
        initIMU();

        //zeroEncoders();

        double oldGyroAngle=readFile(GYRO_ANGLE_FILE_NAME); //get the angle the robot was at when auton ended
        angleZzeroValue=oldGyroAngle;
        writeFile(GYRO_ANGLE_FILE_NAME, 0); //reset the old angle to zero

        telemetry.log().add("Ready to start");
        //telemetry.log().add("gyro angle during auton: " + oldGyroAngle);
        telemetry.log().add("gyro angle: " + getIMUAngle());

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) speedModifer=.3;
            if (gamepad1.b && !gamepad1.start) speedModifer=.7;
            if (gamepad1.a && !gamepad1.start) speedModifer=1;

            // if (gamepad1.a) LeftFront.setPower(1);
            // if (gamepad1.b) RightFront.setPower(1);
            // if (gamepad1.x) LeftBack.setPower(1);
            // if (gamepad1.y) RightBack.setPower(1);

            if (feildRealtive)
            {
                double speed = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                double LeftStickAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double robotAngle = Math.toRadians(getIMUAngle());
                double rightX=gamepad1.right_stick_x;
                rightX*=.5;
                leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;
                rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
                leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
                rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;

                if (gamepad1.x && !isGamepadPressed)
                {
                    feildRealtive = false;
                    isGamepadPressed=true;
                } else if (!gamepad1.x && isGamepadPressed) isGamepadPressed=false;

            }
            else
            {
                leftFrontPower=gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                rightFrontPower=gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                leftBackPower=gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
                rightBackPower=gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

                if (gamepad1.x && !isGamepadPressed) //
                {
                    feildRealtive = true;
                    isGamepadPressed=true;
                } else if (!gamepad1.x && isGamepadPressed) isGamepadPressed=false;

            }

            LeftFront.setPower(leftFrontPower*speedModifer); //set the power according to the speed modifer
            RightFront.setPower(rightFrontPower*speedModifer);
            LeftBack.setPower(leftBackPower*speedModifer);
            RightBack.setPower(rightBackPower*speedModifer);

            if (gamepad2.y && !isXPressed && targetSpinnerPos>=.5 && isLiftHighEnoungh())
            {
                targetSpinnerPos=0;
                //setSpinner(targetSpinnerPos);
                isXPressed=true;
            } else if (!gamepad1.y && isXPressed) isXPressed=false;

            if (gamepad2.y && !isXPressed && targetSpinnerPos<.5 && isLiftHighEnoungh())
            {
                targetSpinnerPos=1;
                //setSpinner(targetSpinnerPos);
                isXPressed=true;
            } else if (!gamepad1.y && isXPressed) isXPressed=false;

/*
            if (gamepad2.a && !isAPressed && getFoundationGrabber() >=.75)
            {
                setFondationGrabber(.5);
                isAPressed=true;
            } else if (!gamepad1.a && isAPressed) isAPressed=false;

            if (gamepad2.a && !isAPressed && getFoundationGrabber() <.75)
            {
                setFondationGrabber(1);
                isAPressed=true;
            } else if (!gamepad2.a && isAPressed) isAPressed=false;
*/

            /* A is not pressed but used to be. Remember that it's not pressed any more */
            if (!gamepad2.a && isAPressed )
            {
                isAPressed=false;
            }

            /* A just got pressed and foundation is "up". Change to "down" and remember that A is pressed */
            if (gamepad2.a && !isAPressed && !foundationGrabberDown)
            {
                setFondationGrabber(0.9);
                isAPressed=true;
                foundationGrabberDown = true;
            }

            /* A just got pressed and foundation is "down". Change to "up" and remember that A is pressed */
            if (gamepad2.a && !isAPressed && foundationGrabberDown)
            {
                setFondationGrabber(0);
                isAPressed=true;
                foundationGrabberDown = false;
            }


            //lift.setPower(-gamepad2.left_stick_y);

            if (gamepad2.right_trigger>=.6 && !isTriggerPressed && getGrabber()>=.5)
            {
                setGrabber(0);
                isTriggerPressed=true;
            }
            else if (isTriggerPressed && gamepad2.right_trigger<.5) isTriggerPressed = false;

            if (gamepad2.right_trigger>=.6 && !isTriggerPressed && getGrabber()<=.5)
            {
                setGrabber(1);
                isTriggerPressed=true;
            }
            else if (isTriggerPressed && gamepad2.right_trigger<.5) isTriggerPressed = false;

            if (gamepad2.x) setGrabber(.25); //set grabber slightly lower

            if (gamepad1.right_bumper && gamepad1.left_bumper) //calibrate gyro
            {
                angleZzeroValue = -angles.firstAngle;
            }

            double r = Math.hypot(gamepad2.right_stick_x, gamepad2.right_stick_y); //uses right stick to spin arm

            if (r>.7 && isLiftHighEnoungh())
            {
                //double c = (2*r) * Math.PI;
                double rightStickAngle = Math.abs(Math.toDegrees(Math.atan2(gamepad2.right_stick_y, gamepad2.right_stick_x)));
                telemetry.addData("right stick angle", rightStickAngle);
                double ratio = rightStickAngle/180;
                //ratio = Math.abs(ratio-1);
                targetSpinnerPos=ratio;
                // if (getGrabber()>= .5) targetSpinnerPos=1;
                // else targetSpinnerPos = 0;
            }
            setSpinner(targetSpinnerPos);

            telemetry.addData("grabber pos", getGrabber());
            telemetry.addData("IMU angle", getIMUAngle());
            telemetry.addData("zero value" , angleZzeroValue);
            telemetry.addData("angles.firstAngle", angles.firstAngle);
            // telemetry.addData("Left Front current Pos", LeftFront.getCurrentPosition());
            // telemetry.addData("Left Back current Pos", LeftBack.getCurrentPosition());
            // telemetry.addData("Right Front current Pos", RightFront.getCurrentPosition());
            // telemetry.addData("Right Back current Pos", RightBack.getCurrentPosition());
            //telemetry.addData("spinner pos", spinner.getPosition());
            telemetry.addData("A is full speed, B is half speed, Y is quarter speed","");
            telemetry.addData("Field Realitive Driving ", feildRealtive);
            // telemetry.addData("Left Trigger", gamepad1.left_trigger);
            // telemetry.addData("Right Trigger", gamepad1.right_trigger);
            // telemetry.addData("Left Front Power",leftFrontPower);
            // telemetry.addData("Right Front Power",rightFrontPower);
            // telemetry.addData("Left Back Power",leftBackPower);
            // telemetry.addData("Right Back Power",rightBackPower);
            telemetry.addData("Speed modifer", speedModifer);
            telemetry.update();
        }
        stopRobot();
    }
}