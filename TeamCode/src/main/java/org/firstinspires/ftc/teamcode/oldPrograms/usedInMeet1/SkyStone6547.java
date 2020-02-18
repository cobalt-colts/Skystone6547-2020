package org.firstinspires.ftc.teamcode.OldPrograms.usedInMeet1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import java.io.File;
import java.util.Locale;
/*
This is the master class with all the methods used in autonomous and tele-op.
 */

@Disabled
public class SkyStone6547 extends LinearOpMode{

    public final double encoderTicksPerRotation=205;
    public final double circumferenceOfWheel=12.566370614359172;

    public double angleZzeroValue=0;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    Acceleration gravity;
    Velocity thing;

    ElapsedTime runtime = new ElapsedTime();

    public final String GYRO_ANGLE_FILE_NAME="gyroAngle.json";

    double deltaX=0;
    double deltaY=0;
    double oldDist=0;

    public DcMotor LeftFront;
    public DcMotor RightFront;
    public DcMotor LeftBack;
    public DcMotor RightBack;

    DcMotor lift;

    Servo spinner;
    Servo grabber;
    Servo fondationGrabber;
    Servo fondationGrabber2;

    public ColorSensor colorSensorSideLeft;
    public ColorSensor colorSensorSideRight;

    public SkyStoneLoc skyStoneLoc=SkyStoneLoc.CENTER; //defualt to center

    int[] levels = new int[] {700,3000}; //linear slide levels, 0 is bottom, 1 is to drop the skystone on top of the fondation, 2 is on top on the first stone, 3 is on top of the second stone, and so one.

    int linMin = 2750;
    public SkyStone6547() {}  //constructor, never used

    public void INIT(HardwareMap hardwareMap)
    {
        LeftBack= hardwareMap.get(DcMotor.class, "Back Left");  //set Motors
        RightBack= hardwareMap.get(DcMotor.class, "Back Right");
        LeftFront= hardwareMap.get(DcMotor.class, "Front Left");
        RightFront = hardwareMap.get(DcMotor.class, "Front Right");
        lift = hardwareMap.get(DcMotor.class, "lift");

        grabber = hardwareMap.get(Servo.class, "grabber"); //set servos
        spinner = hardwareMap.get(Servo.class, "spinner");
        fondationGrabber = hardwareMap.get(Servo.class, "f grabber");
        fondationGrabber2 = hardwareMap.get(Servo.class, "f grabber1");

        colorSensorSideRight = hardwareMap.get(ColorSensor.class, "color sensor2"); //set color sensors
        colorSensorSideLeft = hardwareMap.get(ColorSensor.class, "color sensor3");

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //set drive train motors to brake when power is 0
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE); //reverse the right motors, so that way seting the drivetrain's motor's power to 1 causes the robot to go forward
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        setSpinner(0); //set spinner to be inside of the robot.
        setGrabber(1); //get grabber to closed position (so robot is inside 18 inches)
        setFondationGrabber(0);
    }
    public void writeFile(String filename, double number)
    {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, Double.toString(number));
            //telemetry.log().add("saved " + number + " in " + filename);
        }
        catch(Exception e)
        {
            // telemetry.log().add("Unable to write " + number + " in " + filename);
        }
    }
    public double readFile(String filename)
    {
        try {
            double output=0;
            File file= AppUtil.getInstance().getSettingsFile(filename);
            output = Double.parseDouble(ReadWriteFile.readFile(file));
            //telemetry.log().add("read " + output + " in " + filename);
            return output;
        }
        catch (Exception e)
        {
            //telemetry.log().add("Unable to read " + filename + ", returning 0");
            return 0;
        }
    }
    public void outputTelemetry()
    {
        telemetry.addData("default IMU angle", angles.firstAngle);
        telemetry.addData("IMU angle with zero Value", getIMUAngle());
        telemetry.addData("Front Left POW", LeftFront.getPower());
        telemetry.addData("Front Right POW", RightFront.getPower());
        telemetry.addData("Back Left POW", LeftBack.getPower());
        telemetry.addData("Back Right POW", RightBack.getPower());
        telemetry.update();
    }
    public void sleep(double seconds)
    {
        runtime.reset();
        while (runtime.seconds()<=seconds && opModeIsActive());
    }
    public boolean isLiftHighEnoungh()
    {
        return lift.getCurrentPosition() > linMin;
    }
    public void setLiftLevel(int level)
    {
        setLiftLevel(level, 1);
    }
    public void setLiftLevel(int level, double power) //sets the lift based on the level the user stated
    {

        double target = levels[level];
        telemetry.log().add("target: " + target );
        telemetry.log().add("current pos: " + lift.getCurrentPosition());
        if (lift.getCurrentPosition() > target)
        {
            telemetry.log().add("going down");
            power=-Math.abs(power);
            lift.setPower(power);
            while (lift.getCurrentPosition() > target && opModeIsActive())
            {
                telemetry.addData("lift power", lift.getPower());
                telemetry.addData("lift pos", lift.getCurrentPosition());
                telemetry.update();
            }
            lift.setPower(0);
        }
        else
        {
            telemetry.log().add("going up");
            telemetry.log().add("target: " + target);
            power=Math.abs(power);
            lift.setPower(power);
            while (opModeIsActive() && lift.getCurrentPosition() < target)
            {
                telemetry.addData("lift power", lift.getPower());
                telemetry.addData("lift pos", lift.getCurrentPosition());
                telemetry.update();
            }
            lift.setPower(0);
        }


    }
    public void setSpinner(double pos)
    {
        //pos = Math.abs(pos-1); //invert position
        double min=.495;
        double max=.63;
        double range = max-min;
        pos*=range;
        pos+=min;
        spinner.setPosition(pos);
    }
    public void setFondationGrabber(double pos)
    {
        double min=0;
        double max=.55;
        double max2=.65;
        double range = max-min;
        double range2 = max2-min;
        double pos2 = (pos*range2) +min;
        pos*=range;
        pos+=min;
        fondationGrabber2.setPosition(pos2);
        fondationGrabber.setPosition(Math.abs(1-pos));

    }
    public double getFoundationGrabber()
    {
        double min=0;
        double max=.55;
        double max2=.65;
        double range = max-min;
        return (fondationGrabber2.getPosition()*range) + min;
    }
    public void setGrabber(double pos)
    {
        double min=0;
        double max=.75;
        double range = max-min;
        pos*=range;
        pos+=min;
        grabber.setPosition(pos);
    }
    public double getGrabber() //return the grabbers position
    {
        double min=0;
        double max=.75;
        double range = max-min;
        double pos = grabber.getPosition();
        pos*=range;
        pos+=min;
        return pos;
    }
    public void grabStone()
    {
        grabber.setPosition(1);
    }
    public void releaseStone()
    {
        grabber.setPosition(0);
    }
    public void TurnPID(double angle, double seconds)
    {
        TurnPID(angle, seconds, 1);
    }
    public void TurnPID(double angle, double seconds, double motorPowerModifer) //use PID to turn the robot
    {
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorPowerModifer=Math.abs(motorPowerModifer); //make sure the robot will always go the right direction
        //MiniPID miniPID = new MiniPID(.004, 0, .1);
        MiniPID miniPID = new MiniPID(.004, 0, .01);
        double angleDifference=angle-getIMUAngle();
        if (Math.abs(angleDifference)>180) //make the angle difference less then 180 to remove unnecessary turning
        {
            angleDifference+=(angleDifference>=0) ? -360 : 360;
        }
        double tempZeroValue=angleZzeroValue;
        angleZzeroValue=0;
        angleZzeroValue=-getIMUAngle();

        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        telemetry.log().add("angle difference " + angleDifference);
        double target=angleDifference;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        telemetry.log().add("target: " + target + "degrees");
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {

            output = miniPID.getOutput(actual, target);
            if (angle>175 || angle <-175) actual = getIMUAngle(true);
            else actual = getIMUAngle();
            turnLeft(output*motorPowerModifer);
            telemetry.addData("power", LeftFront.getPower());
            telemetry.addData("angle", actual);
            telemetry.update();
            //outputTelemetry();
        }
        stopRobot();
        angleZzeroValue=tempZeroValue;

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double getIMUAngle() //gets the gyro angle
    {
        double currentAngle=angles.firstAngle+angleZzeroValue;
        while (currentAngle>180) currentAngle-=360;
        while (currentAngle<-180) currentAngle+=360;
        telemetry.addData("current Angle", currentAngle);
        return currentAngle;
    }
    public double getIMUAngle(boolean extendBeyond180)
    {
        double currentAngle=angles.firstAngle+angleZzeroValue;
        if (extendBeyond180) return currentAngle;
        else return getIMUAngle();
    }
    public void DriveforLength(double feet,double power) //drive forward or backward for a distance in feet.
    {
        telemetry.log().add("Drving for length");
        if (power>=0) feet = Math.abs(feet);
        else feet = -Math.abs(feet);
        zeroEncoders();
        driveForward(power);
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        if (drivingDistanceInEncoderTicks>=0) while (RightBack.getCurrentPosition() <=drivingDistanceInEncoderTicks && opModeIsActive())
        {
            telemetry.addData("Encoder ticks to drive (positve)", drivingDistanceInEncoderTicks);
            telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        else while (RightBack.getCurrentPosition() >=drivingDistanceInEncoderTicks && opModeIsActive())
        {
            telemetry.addData("Encoder ticks to drive (negitive)", drivingDistanceInEncoderTicks);
            telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        stopRobot();
        telemetry.log().add("Done with Drive for length");
        telemetry.log().add("Left Front Encoder Pos:"+ LeftFront.getCurrentPosition());
        telemetry.log().add("Right Front Encoder Pos:"+ RightFront.getCurrentPosition());
        telemetry.log().add("Left Back Encoder Pos:"+ LeftBack.getCurrentPosition());
        telemetry.log().add("Right Back Encoder Pos:"+ RightBack.getCurrentPosition());
        telemetry.log().add("Done with Drive for length");
    }
    public void DriveFieldRealtiveDistanceAndTurnPID(double power, double angleInDegrees, double feet, double targetAngle, double rightX) //drive field reative and turn at the same time, however it makes the robot's position slightly unstable
    {
        zeroEncoders();
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        double speed = power;
        double desiredAngle =Math.toRadians(angleInDegrees)-Math.PI / 4;
        //double robotAngle = Math.toRadians(getIMUAngle());
        telemetry.log().add("averge encoder" + averageDrivetrainEncoder());
        while (opModeIsActive() && Math.abs(averageDrivetrainEncoder())<Math.abs(drivingDistanceInEncoderTicks))
        {
            if (getIMUAngle()>=targetAngle+5) rightX=Math.abs(rightX);
            else if (getIMUAngle()<=targetAngle-5) rightX=-Math.abs(rightX);
            else rightX=0;
            double robotAngle = Math.toRadians(getIMUAngle());
            LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            outputTelemetry();
        }
        stopRobot();

    }
    public void DriveFieldRealtiveDistance(double power, double angleInDegrees, double feet, boolean brake)
    {
        angleInDegrees+=180;
        zeroEncoders();
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        double speed = power;
        double desiredAngle =Math.toRadians(angleInDegrees)-Math.PI / 4;
        //double robotAngle = Math.toRadians(getIMUAngle());
        double rightX = 0;
        telemetry.log().add("averge encoder" + averageDrivetrainEncoder());
        while (opModeIsActive() && Math.abs(averageDrivetrainEncoder())<Math.abs(drivingDistanceInEncoderTicks))
        {
            double robotAngle = Math.toRadians(getIMUAngle());
            LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            outputTelemetry();
        }
        if (!brake) //let the motors coast when the robot is stoped
        {
            LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        stopRobot();
        if (!brake) //revert the changes.
        {
            LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }
    public void DriveFieldRealtiveDistance(double power, double angleInDegrees, double feet)
    {
        DriveFieldRealtiveDistance(power, angleInDegrees, feet, true);
    }
    public void DriveFieldRealtiveSimple(double power, double angle) //set the robot's direction based on the gyro.
    {
        power=Math.abs(power);
        angle+=180;
        double angleInRadians =Math.toRadians(angle)-Math.PI / 4;
        double robotAngle = Math.toRadians(getIMUAngle());
        double rightX = 0;
        LeftFront.setPower(power * Math.cos(angleInRadians-robotAngle) + rightX);
        RightFront.setPower(power * Math.sin(angleInRadians-robotAngle) - rightX);
        LeftBack.setPower(power * Math.sin(angleInRadians-robotAngle) + rightX);
        RightBack.setPower(power * Math.cos(angleInRadians-robotAngle) - rightX);

    }
    void driveCircle(double pow, double r, double toAngle, boolean left, boolean forward)
    {
        zeroEncoders();
        double midDist = (r-.75)* Math.PI * (toAngle/180)*(encoderTicksPerRotation/circumferenceOfWheel)*12;
        double leftDist = r * Math.PI * (toAngle/180);
        double rightDist = (r-1.5) * Math.PI * (toAngle/180);

        double leftPow = 1; // leftDist/leftDist
        double rightPow = rightDist/leftDist;
        if (!left)
        {
            leftPow+=rightPow; //swap left and right, no temp variable because I can
            rightPow=leftPow-rightPow;
            leftPow-=rightPow;
        }
        if (forward)
        {
            leftPow=-Math.abs(leftPow);
            rightPow=-Math.abs(rightPow);
        }
        leftPow*=pow;
        rightPow*=pow;
        while (opModeIsActive() && averageDrivetrainEncoder() <= midDist)
        {
            steerRobot(leftPow, rightPow);
        }
        stopRobot();
    }
    double getCircleCircumfrence(double r)
    {
        return (r*2) * Math.PI;
    }
    void driveEllipse(double pow,double a, double b, double toAngle, boolean left, boolean forward)
    {
        driveEllipse(pow,a,b,0,toAngle, left, forward);
    }
    void driveEllipse(double pow, double a, double b, double fromAngle, double toAngle, boolean left, boolean forward)
    {
        double midDist = (getEllipseCircumference(a-.5,b-.5))*((toAngle-fromAngle)/360)*12*205;
        double leftDist = (getEllipseCircumference(a, b))*((toAngle-fromAngle)/360);
        double rightDist = (getEllipseCircumference(a-1, b-1))*((toAngle-fromAngle)/360);

        double leftPow = 1; // leftDist/leftDist
        double rightPow = rightDist/leftDist;
        leftPow*=pow; //left pow is a constant.
        resetDetlaPos();
        while (opModeIsActive() && averageDrivetrainEncoder() <= midDist)
        {
            updateDistance();
            double r = getCircleRaduis(getEllipseCurvature(deltaX));
            double leftCir = getCircleCircumfrence(r+.5);
            double rightCir = getCircleCircumfrence(r-.5);
            rightPow = rightCir/leftCir;
            rightPow*=pow;
            if (!left) //if not going left, reflect the powers.  Goes opposite direction
            {
                leftPow = rightPow;
                rightPow = pow;
            }
            if (!forward)
            {
                leftPow=-Math.abs(leftPow);
                rightPow=-Math.abs(rightPow);
            }
            steerRobot(leftPow, rightPow);
            telemetry.addData("X", deltaX);
            telemetry.addData("Y", deltaY);
            telemetry.addData("left pow", LeftFront.getPower());
            telemetry.addData("right pow", RightBack.getPower());
            telemetry.update();
        }
        stopRobot();
    }
    double getCircleRaduis(double angle)
    {
        return 1/angle;
    }
    double getEllipseCurvature(double x)
    {
        double c = 2500*Math.abs(1/Math.pow(Math.abs((116*Math.pow(x, 2))-625), 1.5));

        return c;
    }
    double getEllipseCurvature(double a, double b, double x)
    {
        double d1 = (Math.pow(b,2) * x)/(Math.pow(Math.abs(((Math.pow(x,2)*Math.pow(b,2))/Math.pow(a,2))-Math.pow(b,2)),.5)*Math.pow(a,2));
        double d2 = -(Math.pow(b,4))/(Math.pow(Math.abs(((Math.pow(x,2)*Math.pow(b,2))/Math.pow(a,2))-Math.pow(b,2)),1.5)*Math.pow(a,2));
        double c = Math.abs(d2)/Math.pow(Math.abs(1+(Math.pow(d1,2))),1.5);
        return c;
    }
    private void updateDistance()
    {
        double dist = averageDrivetrainEncoder()-oldDist;
        deltaX+=dist*Math.cos(Math.toRadians(angles.firstAngle));
        deltaY+=dist*Math.sin(Math.toRadians(angles.firstAngle));
        oldDist+=dist;

    }
    public void resetDetlaPos()
    {
        deltaY=0;
        deltaX=0;
    }
    double getEllipseAngle(double a, double b, double x, double leeway)
    {
        double x1 = x+leeway;
        double x2 = x-leeway;
        double y1 = calcy(a,b,x1);
        double y2 = calcy(a,b,x2);
        return Math.toDegrees(Math.atan((y2-y1)/(x2-x1)));
    }
    double calcy(double a, double b, double x)
    {
        return Math.sqrt(((Math.pow(x,2)/Math.pow(a,2))*Math.pow(b,2))-Math.pow(b,2));
    }
    double getEllipseCircumference(double a, double b)
    {
        double c = (Math.PI * (a+b))*(3*(Math.pow(a-b,2)/(Math.pow(a+b,2)*(Math.sqrt((-3*(Math.pow(a-b,2)/Math.pow(a+b,2)))+4)+10)))+1);
        return c;
    }
    public double averageDrivetrainEncoder() //average the position from the drive train motors
    {
        double motorposition=0;
        motorposition+=Math.abs(LeftBack.getCurrentPosition());
        motorposition+=Math.abs(LeftFront.getCurrentPosition());
        motorposition+=Math.abs(RightBack.getCurrentPosition());
        motorposition+=Math.abs(RightFront.getCurrentPosition());
        if (Double.isNaN(motorposition/4)) return 0;
        else return Math.abs(motorposition/4);
    }
    public void zeroEncoders() //set all the encoders to zero
    {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void DriveToTargetPos(double power, int pos)
    {
        setTargetPos(pos);
        zeroEncoders();
        driveForward(power);
        while (opModeIsActive() && motorsRunning())
        {
            stopEachMotorWhenHitPos();
            outputTelemetry();
        }
        stopRobot();
    }
    void stopEachMotorWhenHitPos()
    {
        if (!LeftFront.isBusy()) LeftFront.setPower(0);
        if (!LeftBack.isBusy()) LeftBack.setPower(0);
        if (!RightFront.isBusy()) RightFront.setPower(0);
        if (!RightBack.isBusy()) RightBack.setPower(0);
    }
    boolean motorsRunning()
    {
        return LeftFront.isBusy() || RightFront.isBusy() || LeftBack.isBusy() || RightBack.isBusy();
    }
    void setTargetPos(int pos)
    {
        LeftFront.setTargetPosition(pos);
        RightFront.setTargetPosition(pos);
        LeftBack.setTargetPosition(pos);
        RightBack.setTargetPosition(pos);
    }
    public void steerRobot(double leftPow, double rightPow)
    {
        LeftFront.setPower(leftPow);
        RightFront.setPower(rightPow);
        LeftBack.setPower(leftPow);
        RightBack.setPower(rightPow);
    }
    public void driveForward(double power)
    {
        LeftFront.setPower(-power);
        RightFront.setPower(-power);
        LeftBack.setPower(-power);
        RightBack.setPower(-power);
    }
    public void turnLeft(double power) {
        LeftFront.setPower(power);
        RightFront.setPower(-power);
        LeftBack.setPower(power);
        RightBack.setPower(-power);
    }
    public void DriveLeft(double power) { //strafe left
        LeftFront.setPower(-power);
        RightFront.setPower(power);
        LeftBack.setPower(power);
        RightBack.setPower(-power);
    }
    public void stopRobot()
    {
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
    }
    public void initIMU() //code to init the IMU, this method is from Sample code that came with the FTC SDK
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();


    }
    void composeTelemetry() { //called in initIMU(), and this method is from Sample code that came with the FTC SDK

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            thing = imu.getVelocity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }{ //called in initIMU()

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            thing = imu.getVelocity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formaftting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void runOpMode() {}
}
