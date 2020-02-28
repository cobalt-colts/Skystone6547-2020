package org.firstinspires.ftc.teamcode.oldPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

/*
This is the tele-op we use to drive the robot
 */
@TeleOp(name = "SkyStone Tele-op meet2")
@Disabled
public class SkyStoneTeleOpMeet2 extends SkyStone6547Meet2 {

    double speedModifer=.7; //lowers the speed so it's easier to drive

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    boolean feildRealtive=true;

    boolean isGamepadPressed=false;//used to program toggle function in the X button in gamepad 1

    boolean isAPressed = false;

    boolean foundationGrabberDown = false;

    @Override
    public void runOpMode() {
        INIT(hardwareMap);
        telemetry.update();
        initIMU();

        zeroEncoders();

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
            
            if (gamepad2.right_trigger >=.6)
            {
                intake(1);
            }
            else if (gamepad2.left_trigger>=.6)
            {
                outtake(1);
            }
            else
            {
                //stopIntake();
            }

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


            lift.setPower(-gamepad2.left_stick_y);

            if (gamepad1.right_bumper && gamepad1.left_bumper) //calibrate gyro
            {
                angleZzeroValue = -angles.firstAngle;
            }

            double r = Math.hypot(gamepad2.right_stick_x, gamepad2.right_stick_y); //uses right stick to spin arm

            telemetry.addData("Left Front Expandtion hub" , getExpansionHub(LeftFront));
            telemetry.addData("Left Back Expandtion hub", getExpansionHub(LeftBack));
            telemetry.addData("Right Front Expantion hub",getExpansionHub(RightFront));
            telemetry.addData("Right Back Expandtion hub",getExpansionHub(RightBack));
            telemetry.addData("Left back connection into",LeftBack.getConnectionInfo());

            telemetry.addData("IMU angle", getIMUAngle());
            telemetry.addData("zero value" , angleZzeroValue);
            telemetry.addData("angles.firstAngle", angles.firstAngle);
            // telemetry.addData("Left Front current Pos", LeftFront.getCurrentPosition());
            // telemetry.addData("Left Back current Pos", LeftBack.getCurrentPosition());
            // telemetry.addData("Right Front current Pos", RightFront.getCurrentPosition());
            // telemetry.addData("Right Back current Pos", RightBack.getCurrentPosition());
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
