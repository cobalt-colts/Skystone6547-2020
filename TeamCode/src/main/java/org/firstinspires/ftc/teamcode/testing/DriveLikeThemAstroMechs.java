package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

@Autonomous(name = "Drive like the AstroMechs do")
@Disabled
public class DriveLikeThemAstroMechs extends SkyStone6547Meet2 {

    protected static final double P_DRIVE_COEFF = 0.042;

    private double wheelDiameter = 4;

    double ticksPerInch = ((encoderTicksPerRotation)/(wheelDiameter * Math.PI));

    @Override
    public void runOpMode() throws InterruptedException {
        INIT(hardwareMap);
        initIMU();
        telemetry.log().add("Ready to start");
        waitForStart();

        driveStraight(30,0,1);
        sleep(1000);
        turn(90,.5);
    }

    //code below is by the 3409 Astromechs that I ungraciously and unprofessionally stole

    public void driveStraight(double inches, float heading, double speedLimit)  throws InterruptedException {
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double leftPower;                                       //Power being fed to left side of bot
        double rightPower;                                      //Power being fed to right side of bot
        double max;                                             //To be used to keep powers from exceeding 1
        double P_COEFF = 0.002;
        double D_COEFF = 0.00042;
        if(inches<24){
            P_COEFF = 0.0015;
            D_COEFF = 0.00022;
        }
        double power;
        double deltaT;
        double derivative = 0;
        int deltaD;
        int lastEncoder;
        int distance;
        long loops = 0;
        heading = (int) normalize360(heading);

        zeroEncoders();
        lastEncoder = (int) averageDrivetrainEncoder();

        int target = (int) (inches * ticksPerInch);

        //speedLimit = Range.clip(speedLimit, -1.0, 1.0);

        if (speedLimit==0){
            return;
        }

        double lastTime = getRuntime();

        while ((((Math.abs(target)-50) > Math.abs(getEncoderWheelPosition()) || ((Math.abs(target)+50) < Math.abs(getEncoderWheelPosition())))
                || (loops==0 || Math.abs(derivative)<80)) && opModeIsActive()) {

            error = heading - normalize360((float) getIMUAngle());

            distance = Math.abs(target) - Math.abs(getEncoderWheelPosition());

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            deltaT = getRuntime()-lastTime;
            lastTime = getRuntime();
            deltaD = getEncoderWheelPosition()-lastEncoder;
            lastEncoder = getEncoderWheelPosition();

            derivative = ((double) deltaD)/deltaT;

            power = (distance*P_COEFF) - (derivative*D_COEFF);

            if (Math.abs(power) > Math.abs(speedLimit)) {
                power /= Math.abs(power);
                power *= Math.abs(speedLimit);
            }

            power *= (speedLimit/Math.abs(speedLimit));

            leftPower = power - correction;
            rightPower = power + correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            updateDriveMotors(leftPower, rightPower, leftPower, rightPower);

            if (((loops+10) % 10) ==  0) {
                telemetry.addData("gyro" , getIMUAngle());
                telemetry.addData("encoder" , getEncoderWheelPosition());
                telemetry.addData("loops", loops);
                telemetry.addData("deltaD", deltaD);
                telemetry.addData("deltaT", deltaT);
                telemetry.addData("distance", distance);
                telemetry.addData("derivative", derivative);
                telemetry.update();
            }

            loops++;

            sleep(10);
        }
    }
    public int getEncoderWheelPosition() {return (int) Math.abs(averageDrivetrainEncoder());}
    public void updateDriveMotors(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower)
    {
        LeftFront.setPower(leftFrontPower);
        RightFront.setPower(rightFrontPower);
        LeftBack.setPower(leftBackPower);
        RightBack.setPower(rightBackPower);
    }
    protected float normalize360(float val) {
        while (val > 360 || val < 0) {

            if (val > 360) {
                val -= 360;
            }

            if (val < 0) {
                val += 360;
            }
        }
        return val;
    }

    public void turn(float turnHeading, double power) throws InterruptedException {
        int wrapFix = 0;                                        //Can be used to modify values and make math around 0 easier
        float shiftedTurnHeading = turnHeading;                 //Can be used in conjunction with wrapFix to make math around 0 easier
        long loops = 0;

        power = Math.abs(power);                                //makes sure the power is positive
        if (power>1) power = 1;                                 //makes sure the power isn't >1

        //If heading is not on correct scale, put it between 0-360
        turnHeading = normalize360(turnHeading);

        //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
        float cclockwise = normalize360((float) getIMUAngle()) - turnHeading;
        float clockwise = turnHeading - normalize360((float) getIMUAngle());

        //Normalize cwise & ccwise values to between 0=360
        clockwise = normalize360(clockwise);
        cclockwise = normalize360(cclockwise);
        int error = 1;

        //sets the distance to the target gyro value that we will accept
        if (turnHeading - error < 0|| turnHeading + error > 360) {
            wrapFix = 180;                                      //if within the range where the clockmath breaks, shift to an easier position
            shiftedTurnHeading = normalize360(turnHeading + wrapFix);
        }

        //If it would be longer to take the ccwise path, we go *** CLOCKWISE ***
        if(Math.abs(cclockwise) >= Math.abs(clockwise)){

            updateDriveMotors(-power, power, -power, power);

            //While we're not within our error, and we haven't overshot, and the bot is running
            while(Math.abs(normalize360(normalize360((float) getIMUAngle()) + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(cclockwise) >= Math.abs(clockwise) && opModeIsActive()) {

                //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
                cclockwise = normalize360((float) getIMUAngle())- turnHeading;
                clockwise = turnHeading - normalize360((float) getIMUAngle());

                //Normalize cwise & ccwise values to between 0=360
                clockwise = normalize360(clockwise);
                cclockwise = normalize360(cclockwise);

                if ((loops % 10) ==  0) {
                    telemetry.addData("gyro" , normalize360((float) getIMUAngle()));
                    telemetry.addData("loops", loops);
                    telemetry.update();
                }

                loops++;

                //Chill a hot decisecond
                Thread.sleep(10);
            }
        }
        //If it would take longer to take the cwise path, we go *** COUNTERCLOCKWISE ***
        else if(Math.abs(clockwise) > Math.abs(cclockwise)) {

            updateDriveMotors(power, -power, power, -power);

            //While we're not within our error, and we haven't overshot, and the bot is running
            while (Math.abs(normalize360(normalize360((float) getIMUAngle()) + wrapFix) - shiftedTurnHeading) > error &&
                    Math.abs(clockwise) > Math.abs(cclockwise) && opModeIsActive()) {

                //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
                cclockwise = normalize360((float) getIMUAngle()) - turnHeading;
                clockwise = turnHeading - normalize360((float) getIMUAngle());

                //Normalize cwise & ccwise values to between 0=360
                clockwise = normalize360(clockwise);
                cclockwise = normalize360(cclockwise);

                if ((loops % 10) ==  0) {
                    telemetry.addData("gyro" , normalize360((float) getIMUAngle()));
                    telemetry.addData("loops", loops);
                    telemetry.update();
                }

                loops++;

                //Hold up a hot decisecond
                Thread.sleep(10);
            }
        }
    }
}
