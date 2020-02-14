package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutionException;

import org.firstinspires.ftc.teamcode.util.state.Button;
/*
This is the master class with all the methods used in autonomous and tele-op.
 */

@Config
@Disabled
public class SkyStone6547Qualifter{

    final double encoderTicksPerRotation=205;
    final double circumferenceOfWheel=12.566370614359172;

    public static double angleZzeroValue=0;

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    Acceleration gravity;
    Velocity thing;

    ElapsedTime runtime = new ElapsedTime();

    public final String GYRO_ANGLE_FILE_NAME="gyroAngle.json";

    double deltaX=0;
    double deltaY=0;
    double oldDist=0;


    public ExpansionHubMotor LeftFront;
    public ExpansionHubMotor RightFront;
    public ExpansionHubMotor LeftBack;
    public ExpansionHubMotor RightBack;

    public ExpansionHubMotor[] driveTrainMotors;
    public double[] driveTrainExpanstionHubNumbers;

    public ExpansionHubMotor lift;
    public ExpansionHubMotor intakeLeft;
    public ExpansionHubMotor intakeRight;

    ExpansionHubServo fondationGrabber;
    ExpansionHubServo fondationGrabber2;
    ExpansionHubServo frontGrabber;
    ExpansionHubServo backGrabber;

    public ColorSensor colorSensorSideLeft;
    public ColorSensor colorSensorSideRight;

    public SkyStoneLoc skyStoneLoc=SkyStoneLoc.CENTER; //defualt to center

    public Button a1,a2,b1,b2,x1,x2,y1,y2,dpadUp1,dpadUp2,dpadDown1, dpadDown2, dpadLeft1, dpadLeft2,dpadRight1, dpadRight2,leftBumper1,leftBumper2,rightBumper1,rightBumper2,start1,start2, rightTrigger1, rightTrigger2, leftTrigger1, leftTrigger2;

    int[] levels = new int[] {700,3000}; //linear slide levels, 0 is bottom, 1 is to drop the skystone on top of the fondation, 2 is on top on the first stone, 3 is on top of the second stone, and so one.

    final int liftMax = 7278;
    final int liftMin = 50;

    RevBulkData bulkData2;
    RevBulkData bulkData3;

    ExpansionHubEx expansionHub2;
    ExpansionHubEx expansionHub3;

    public OpMode opMode;

    double lastPosX;
    double lastPosY;

    Rev2mDistanceSensor distanceSensorX;
    Rev2mDistanceSensor distanceSensorY;

    List<Integer> screamIds = new ArrayList<>();

    ElapsedTime screamTime = new ElapsedTime();

    HardwareMap hardwareMap;

    public SkyStone6547Qualifter(OpMode _opMode) {
        INIT(_opMode);
    }  //constructor, never used



    public void INIT(OpMode _opMode)
    {
        this.opMode = _opMode;
        hardwareMap = opMode.hardwareMap;
        distanceSensorX = hardwareMap.get(Rev2mDistanceSensor.class, "d boi");
        distanceSensorY = hardwareMap.get(Rev2mDistanceSensor.class, "d boi1");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHub3 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        LeftBack= (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "leftBack");  //set Motors
        RightBack= (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "rightBack");
        LeftFront= (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "leftFront");
        RightFront = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "rightFront");
        lift = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "lift");
        intakeLeft = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "intake left");
        intakeRight = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "intake right");
        fondationGrabber = (ExpansionHubServo) hardwareMap.get(Servo.class, "f grabber");
        fondationGrabber2 = (ExpansionHubServo) hardwareMap.get(Servo.class, "f grabber1");
        backGrabber = (ExpansionHubServo) hardwareMap.get(Servo.class, "back grabber");
        frontGrabber = (ExpansionHubServo) hardwareMap.get(Servo.class, "front grabber");

        colorSensorSideRight = hardwareMap.get(ColorSensor.class, "color sensor"); //set color sensors
        colorSensorSideLeft = hardwareMap.get(ColorSensor.class, "color sensor2");

        opMode.telemetry.log().add("Initialized hardware");

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //set drive train motors to brake when power is 0
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE); //reverse the right motors, so that way seting the drivetrain's motor's power to 1 causes the robot to go forward
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        opMode.telemetry.log().add("set hardware");

        setFondationGrabber(0);

        initIMU();

        opMode.telemetry.log().add("Initialized IMU");

        initGamepads();

        opMode.telemetry.log().add("Initialized gamepads");

        driveTrainMotors  = new ExpansionHubMotor[] {LeftBack, LeftFront, RightBack, RightFront};
        driveTrainExpanstionHubNumbers = new double[driveTrainMotors.length];

        setExpansionHubNumbers();

        opMode.telemetry.log().add("Assigned expansion numbers");

        holdFromFrontGrabber();


            screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream1",   "raw", hardwareMap.appContext.getPackageName()));
            screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream2",   "raw", hardwareMap.appContext.getPackageName()));
            screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream3",   "raw", hardwareMap.appContext.getPackageName()));
            screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream4",   "raw", hardwareMap.appContext.getPackageName()));
            screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream5",   "raw", hardwareMap.appContext.getPackageName()));
            screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream6",   "raw", hardwareMap.appContext.getPackageName()));
        screamTime.reset();

        //opMode.telemetry = dashboard.getopMode.telemetry();
    }
    private void initGamepads()
    {
        a1 = new Button();
        a2 = new Button();
        b1 = new Button();
        b2 = new Button();
        x1 = new Button();
        x2 = new Button();
        y1 = new Button();
        y2 = new Button();
        dpadDown1 = new Button();
        dpadDown2 = new Button();
        dpadLeft1 = new Button();
        dpadLeft2 = new Button();
        dpadRight1 = new Button();
        dpadRight2 = new Button();
        dpadUp1 = new Button();
        dpadUp2 = new Button();
        rightBumper1 = new Button();
        rightBumper2 = new Button();
        leftBumper1 = new Button();
        leftBumper2 = new Button();
        start1=new Button();
        start2=new Button();
        leftTrigger1 = new Button();
        leftTrigger2 = new Button();
        rightTrigger1 = new Button();
        rightTrigger2 = new Button();
    }
    public void updateGamepads()
    {
        a1.input(opMode.gamepad1.a);
        a2.input(opMode.gamepad2.a);
        b1.input(opMode.gamepad1.b);
        b2.input(opMode.gamepad2.b);
        x1.input(opMode.gamepad1.x);
        x2.input(opMode.gamepad2.x);
        y1.input(opMode.gamepad1.y);
        y2.input(opMode.gamepad2.y);
        dpadUp1.input(opMode.gamepad1.dpad_up);
        dpadUp2.input(opMode.gamepad2.dpad_up);
        dpadRight1.input(opMode.gamepad1.dpad_right);
        dpadRight2.input(opMode.gamepad2.dpad_right);
        dpadUp1.input(opMode.gamepad1.dpad_up);
        dpadUp2.input(opMode.gamepad2.dpad_up);
        dpadLeft1.input(opMode.gamepad1.dpad_left);
        dpadLeft2.input(opMode.gamepad2.dpad_left);
        leftBumper1.input(opMode.gamepad1.left_bumper);
        leftBumper2.input(opMode.gamepad2.left_bumper);
        rightBumper1.input(opMode.gamepad1.right_bumper);
        rightBumper2.input(opMode.gamepad2.right_bumper);
        leftTrigger1.input(opMode.gamepad1.left_trigger>=.7);
        leftTrigger2.input(opMode.gamepad2.left_trigger>=.7);
        rightTrigger1.input(opMode.gamepad1.right_trigger>=.7);
        rightTrigger2.input(opMode.gamepad2.right_trigger>=.7);

    }
    public void writeFile(String filename, double number)
    {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, Double.toString(number));
            //opMode.telemetry.log().add("saved " + number + " in " + filename);
        }
        catch(Exception e)
        {
            // opMode.telemetry.log().add("Unable to write " + number + " in " + filename);
        }
    }
    public double readFile(String filename)
    {
        try {
            double output=0;
            File file= AppUtil.getInstance().getSettingsFile(filename);
            output = Double.parseDouble(ReadWriteFile.readFile(file));
            //opMode.telemetry.log().add("read " + output + " in " + filename);
            return output;
        }
        catch (Exception e)
        {
            //opMode.telemetry.log().add("Unable to read " + filename + ", returning 0");
            return 0;
        }
    }
    public void outputTelemetry()
    {
        opMode.telemetry.addData("default IMU angle", angles.firstAngle);
        opMode.telemetry.addData("IMU angle with zero Value", getIMUAngle());
//        opMode.telemetry.addData("Front Left POW", LeftFront.getPower());
//        opMode.telemetry.addData("Front Right POW", RightFront.getPower());
//        opMode.telemetry.addData("Back Left POW", LeftBack.getPower());
//        opMode.telemetry.addData("Back Right POW", RightBack.getPower());
        opMode.telemetry.update();
    }
    public void sleep(double seconds)
    {
        runtime.reset();
        while (runtime.seconds()<=seconds && ((LinearOpMode) opMode).opModeIsActive())
        {
            opMode.telemetry.addData("Status","Sleeping for " + (seconds-runtime.seconds()) + " seconds");
        }
    }
    // public boolean isLiftHighEnoungh()
    // {
    //     return lift.getCurrentPosition() > linMin;
    // }
    public void intake(double pow)
    {
        intakeLeft.setPower(pow);
        intakeRight.setPower(-pow);
        opMode.telemetry.log().add("intaking");
    }
    public void outtake(double pow)
    {
        intakeLeft.setPower(-pow);
        intakeRight.setPower(pow);
        opMode.telemetry.log().add("outtaking");
    }
    public void stopIntake()
    {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        opMode.telemetry.log().add("stoppe intake");
    }
    public void releaseGrabbers()
    {
        setFrontGrabber(0);
        setBackGrabber(1);
    }
    public void grabBlock()
    {
        setFrontGrabber(1);
        setBackGrabber(1);
    }
    public void holdFromBackGrabber()
    {
        setFrontGrabber(0);
        setBackGrabber(1);
    }
    public void holdFromFrontGrabber()
    {
        setFrontGrabber(1);
        setBackGrabber(0);
    }
    public void setFrontGrabber(double pos)
    {
        double min=0;
        double max=.44;
        double range = max-min;
        pos*=range;
        pos+=min;
        frontGrabber.setPosition(pos);
    }
    public void setBackGrabber(double pos)
    {
        pos = Math.abs(1-pos); //invert pos
        double min=.18;
        double max=.635;
        double range = max-min;
        pos*=range;
        pos+=min;
        backGrabber.setPosition(pos);
    }
    public void setLiftLevel(int level)
    {
        setLiftLevel(level, 1);
    }
    public void setLiftLevel(int level, double power) //sets the lift based on the level the user stated
    {

        double target = levels[level];
        opMode.telemetry.log().add("target: " + target );
        opMode.telemetry.log().add("current pos: " + lift.getCurrentPosition());
        if (lift.getCurrentPosition() > target)
        {
            opMode.telemetry.log().add("going down");
            power=-Math.abs(power);
            lift.setPower(power);
            while (lift.getCurrentPosition() > target && ((LinearOpMode) opMode).opModeIsActive())
            {
                opMode.telemetry.log().add("Status","Moving Lift Down");
                opMode.telemetry.addData("lift power", lift.getPower());
                opMode.telemetry.addData("lift pos", lift.getCurrentPosition());
                outputTelemetry();
            }
            lift.setPower(0);
        }
        else
        {
            opMode.telemetry.log().add("going up");
            opMode.telemetry.log().add("target: " + target);
            power=Math.abs(power);
            lift.setPower(power);
            while (((LinearOpMode) opMode).opModeIsActive() && lift.getCurrentPosition() < target)
            {
                opMode.telemetry.addData("Status","Moving Up");
                opMode.telemetry.addData("lift power", lift.getPower());
                opMode.telemetry.addData("lift pos", lift.getCurrentPosition());
                outputTelemetry();
            }
            lift.setPower(0);
        }


    }
    public void setFondationGrabber(double pos)
    {
        double min=0;
        double max=.55;
        double max2 = max+.18;
        double range = max-min;
        double range2 = max2-min;
        double pos2 = (pos*range2) +min;
        pos*=range;
        pos+=min;
        fondationGrabber2.setPosition(pos2);
        fondationGrabber.setPosition(Math.abs(1-pos));
        opMode.telemetry.log().add("set foundation grabber to "  + pos);

    }
    public double getFoundationGrabber()
    {
        double min=0;
        double max=.55;
        double max2=.65;
        double range = max-min;
        return (fondationGrabber2.getPosition()*range) + min;
    }
    public void TurnPID(double angle, double seconds)
    {
        TurnPID(angle, seconds, 1);
    }
    public void TurnPID(double angle, double seconds, double motorPowerModifer) { TurnPID(angle, seconds, motorPowerModifer, new MiniPID(.01,0.00001,0.015));}
    public void TurnPID(double angle, double seconds, double motorPowerModifer, MiniPID miniPID) //use PID to turn the robot
    {
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorPowerModifer=Math.abs(motorPowerModifer); //make sure the robot will always go the right direction
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
        //opMode.telemetry.log().add("PID angle difference " + angleDifference);
        double target=angleDifference;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        opMode.telemetry.log().add("PID target angle: " + target + "degrees");
        runtime.reset();
        while (((LinearOpMode) opMode).opModeIsActive() && runtime.seconds() < seconds) {

            output = miniPID.getOutput(actual, target);
            //if (angle>175 || angle <-175) actual = getIMUAngle(true);
            actual = getIMUAngle();
            turnLeft(output*motorPowerModifer);
            opMode.telemetry.addData("Status","Turning PID");
            opMode.telemetry.addData("power", LeftFront.getPower());
            opMode.telemetry.addData("angle", actual);
            opMode.telemetry.update();
            outputTelemetry();
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
        return currentAngle;
    }
    public double getIMUAngle(boolean extendBeyond180)
    {
        double currentAngle=angles.firstAngle+angleZzeroValue;
        if (extendBeyond180) return currentAngle;
        else return getIMUAngle();
    }
    public boolean isSkystone(ColorSensor colorSensorToBeUsed)
    {
        //return colorSensorToBeUsed.red() <65;
        return (colorSensorToBeUsed.red()*colorSensorToBeUsed.green()) / Math.pow(colorSensorToBeUsed.blue(),2) < 3;
    }
    public void DriveforLength(double feet,double power) //drive forward or backward for a distance in feet.
    {
        opMode.telemetry.log().add("Drving for length");
        if (power>=0) feet = Math.abs(feet);
        else feet = -Math.abs(feet);
        zeroEncoders();
        driveForward(power);
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        if (drivingDistanceInEncoderTicks>=0) while (RightBack.getCurrentPosition() <=drivingDistanceInEncoderTicks && ((LinearOpMode) opMode).opModeIsActive())
        {
            opMode.telemetry.addData("Status","Driving For Length");
            opMode.telemetry.addData("Encoder ticks to drive (positve)", drivingDistanceInEncoderTicks);
            opMode.telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        else while (RightBack.getCurrentPosition() >=drivingDistanceInEncoderTicks && ((LinearOpMode) opMode).opModeIsActive())
        {
            opMode.telemetry.addData("Status","Driving For Length");
            opMode.telemetry.addData("Encoder ticks to drive (negitive)", drivingDistanceInEncoderTicks);
            opMode.telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        stopRobot();
        opMode.telemetry.log().add("Done with Drive for length");
        opMode.telemetry.log().add("Left Front Encoder Pos:"+ LeftFront.getCurrentPosition());
        opMode.telemetry.log().add("Right Front Encoder Pos:"+ RightFront.getCurrentPosition());
        opMode.telemetry.log().add("Left Back Encoder Pos:"+ LeftBack.getCurrentPosition());
        opMode.telemetry.log().add("Right Back Encoder Pos:"+ RightBack.getCurrentPosition());
        opMode.telemetry.log().add("Done with Drive for length");
    }
    public void DriveFieldRealtiveDistanceAndTurnPID(double power, double drivingAngle, double feet, double turningAngle, double rightX) //drive field reative and turn at the same time, however it makes the robot's position slightly unstable
    {
        zeroEncoders();

        //turning calculations

        rightX=Math.abs(rightX); //make sure the robot will always go the right direction
        MiniPID miniPID = new MiniPID(.004, 0, .036);
        double angleDifference=turningAngle-getIMUAngle();
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
        opMode.telemetry.log().add("PID angle difference " + angleDifference);
        double target=angleDifference;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        opMode.telemetry.log().add("PID target angle: " + target + "degrees");

        //encoder calculations

        drivingAngle-=tempZeroValue;
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        double speed = power;
        double desiredAngle =Math.toRadians(drivingAngle)-Math.PI / 4;
        //double robotAngle = Math.toRadians(getIMUAngle());
        opMode.telemetry.log().add("average encoder" + averageDrivetrainEncoder());
        while (((LinearOpMode) opMode).opModeIsActive() && Math.abs(averageDrivetrainEncoder())<Math.abs(drivingDistanceInEncoderTicks))
        {
            rightX = miniPID.getOutput(getIMUAngle());
            double robotAngle = Math.toRadians(getIMUAngle());
            LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            opMode.telemetry.addData("Status", "Driving Field Realtive and turning PID");
            outputTelemetry();
        }
        stopRobot();
        angleZzeroValue = tempZeroValue;

    }
    public void DriveFieldRealtiveDistance(double power, double angleInDegrees, double feet, boolean brake)
    {
        power/=2;
        angleInDegrees+=180;
        zeroEncoders();
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        double speed = power;
        double desiredAngle =Math.toRadians(angleInDegrees)-Math.PI / 4;
        //double robotAngle = Math.toRadians(getIMUAngle());
        double rightX = 0;
        opMode.telemetry.log().add("averge encoder" + averageDrivetrainEncoder());
        while (((LinearOpMode) opMode).opModeIsActive() && Math.abs(averageDrivetrainEncoder())<Math.abs(drivingDistanceInEncoderTicks))
        {
            double robotAngle = Math.toRadians(getIMUAngle());
            LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            opMode.telemetry.addData("Status","Driving Field Relative");
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
        while (((LinearOpMode) opMode).opModeIsActive() && averageDrivetrainEncoder() <= midDist)
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
        while (((LinearOpMode) opMode).opModeIsActive() && averageDrivetrainEncoder() <= midDist)
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
            opMode.telemetry.addData("X", deltaX);
            opMode.telemetry.addData("Y", deltaY);
            opMode.telemetry.addData("left pow", LeftFront.getPower());
            opMode.telemetry.addData("right pow", RightBack.getPower());
            opMode.telemetry.update();
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
    public DcMotor zeroEncoder(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }
    void DriveToTargetPos(double power, int pos)
    {
        setTargetPos(pos);
        zeroEncoders();
        driveForward(power);
        while (((LinearOpMode) opMode).opModeIsActive() && motorsRunning())
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
    public void runMotor(DcMotor motor, double pow, int target) throws InterruptedException, ExecutionException
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoder
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(.4);
        while (((LinearOpMode) opMode).opModeIsActive() && Math.abs(motor.getCurrentPosition()) < Math.abs(target))
        {
            motor.setPower(pow);
        }
        motor.setPower(0);
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
    public void scream()
    {
        if (screamTime.seconds() >= 2)
        {
            int soundToPlay = (int) (Math.random() * screamIds.size());

            if (screamIds.get(soundToPlay) != 0)
            {
                SoundPlayer.getInstance().preload(hardwareMap.appContext, screamIds.get(soundToPlay));
            }
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, screamIds.get(soundToPlay));
            screamTime.reset();
        }
        else
        {
            opMode.telemetry.log().add("out of time");
        }
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

        opMode.telemetry.addData("Status", "Initialized IMU");


    }
    void composeTelemetry() { //called in initIMU(), and this method is from Sample code that came with the FTC SDK

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        opMode.telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            thing = imu.getVelocity();
        }
        });
        opMode.telemetry.addLine()
                .addData("Status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        opMode.telemetry.addLine()
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
        opMode.telemetry.addLine()
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
        //telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
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


    //------------------------------------------------------
    //
    //                  REV BULK DATA METHODS
    //
    //------------------------------------------------------

    //public double averageDrivetrainEncoder() //average the position from the drive train motors OLD
    //    {
//        double motorposition=0;
//        motorposition+=Math.abs(LeftBack.getCurrentPosition());
//        motorposition+=Math.abs(LeftFront.getCurrentPosition());
//        motorposition+=Math.abs(RightBack.getCurrentPosition());
//        motorposition+=Math.abs(RightFront.getCurrentPosition());
//        if (Double.isNaN(motorposition/4)) return 0;
//        else return Math.abs(motorposition/4);
//    }
    public double averageDrivetrainEncoder()
    {
        double motorPos = 0;
        updateBulkData();
        for (int i = 0; i < driveTrainMotors.length; i++)
        {
            if (driveTrainExpanstionHubNumbers[i]==2) motorPos+=Math.abs(bulkData2.getMotorCurrentPosition(driveTrainMotors[i]));
            else motorPos+=Math.abs(bulkData3.getMotorCurrentPosition(driveTrainMotors[i]));
        }
        return motorPos/driveTrainMotors.length;
    }
    public void updateBulkData()
    {
        bulkData2=expansionHub2.getBulkInputData();
        bulkData3=expansionHub3.getBulkInputData();
    }
    public int getExpansionHub(DcMotor motor)
    {
        String numString = motor.getConnectionInfo().split(";")[1].split(" ")[2]; //string of lots of stuff.  Split it to get Expanstion hub data
        return Integer.valueOf(numString);
    }
    public void setExpansionHubNumbers()
    {
        for (int i=0;i<driveTrainMotors.length;i++)
        {
            driveTrainExpanstionHubNumbers[i]=getExpansionHub(driveTrainMotors[i]);
        }
    }

    public void DriveToPointPID(double x, double y, double seconds) { DriveToPointPID(x,y,seconds,-90);}
    public void DriveToPointPID(double x, double y, double seconds,double offset)
    {
        opMode.telemetry.log().add("Driving PID");
        MiniPID miniPID = new MiniPID(.05, 0.000, 0.00);
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(x);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actualX=0;
        double outputX=0;

        MiniPID miniPIDY = new MiniPID(.10, 0.00, 0.05);
        miniPIDY.setSetpoint(0);
        miniPIDY.setSetpoint(y);
        miniPIDY.setOutputLimits(1);

        miniPIDY.setSetpointRange(40);

        double actualY=0;
        double outputY=0;

        runtime.reset();
        while (((LinearOpMode) opMode).opModeIsActive() && runtime.seconds()<seconds) {
            actualX = getRobotPositionX();
            outputX = miniPID.getOutput(actualX, x);
            actualY = getRobotPositionY();
            outputY = miniPIDY.getOutput(actualY, y);
            double power = Math.hypot(outputX, outputY);
            //if (power>.8) power = .8; //set max power as .8
            double slope = getSlope(x, y, actualX, actualY);
            double angle = Math.toDegrees(Math.atan2(outputX, -outputY)) + offset;
            DriveFieldRealtiveSimple(power, angle);
            if (Math.abs(getIMUAngle())>=5)
            {
                TurnPID(0,1);
                seconds++;
            }
            opMode.telemetry.addData("X pos" , actualX);
            opMode.telemetry.addData("y pos" , actualY);
            opMode.telemetry.update();
        }
        stopRobot();
    }
    public void strafeToDistanceXPID(double inch, double time) {strafeToDistanceXPID(inch,time,-90);}
    public void strafeToDistanceXPID(double inch, double time, double offset)
    {
        MiniPID miniPID = new MiniPID(.03, 0.000, 0.00);
        double target = inch;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        runtime.reset();
        while (((LinearOpMode) opMode).opModeIsActive() && runtime.seconds()<time) {

            actual = getRobotPositionX();
            output = miniPID.getOutput(actual, target);
            DriveFieldRealtiveSimple(output, (output>=0) ? 90+offset : 270+offset);
            opMode.telemetry.addData("Output", output);
            opMode.telemetry.addData("Pos X", getRobotPositionX());
            opMode.telemetry.update();
        }

    }
    /*
    void strafeToDistanceYPID(double inch, double gap)
    {
        MiniPID miniPID = new MiniPID(.10, 0.00, 0.05);
        double target = inch;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        miniPID.setOutputLimits(1);
        miniPID.setSetpointRange(40);
        double actual=0;
        double output=0;
        runtime.reset();
        while (((LinearOpMode) opMode).opModeIsActive() && runtime.seconds()<gap) {
            actual = getRobotPositionY();
            output = miniPID.getOutput(actual, target);
            DriveFieldRealtiveSimple(output, (output>=0) ? 180 : 0);
            opMode.telemetry.addData("Output", output);
            opMode.telemetry.addData("Pos Y", getRobotPositionY());
            opMode.telemetry.update();
        }
        stopRobot();
    }
    */
    public void strafeToDistanceYPID(double inch, double gap) { strafeToDistanceYPID(inch,gap,-90);}
    public void strafeToDistanceYPID(double inch, double gap, double offset)
    {
        MiniPID miniPID = new MiniPID(.03, 0.00, 0.0);
        double target = inch;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        runtime.reset();
        while (((LinearOpMode) opMode).opModeIsActive() && runtime.seconds()<gap) {

            actual = getRobotPositionY();
            output = miniPID.getOutput(actual, target);
            DriveFieldRealtiveSimple(output, (output>=0) ? 180+offset : 0+offset);
            opMode.telemetry.addData("Output", output);
            opMode.telemetry.addData("Pos Y", getRobotPositionY());
            opMode.telemetry.update();
        }
        stopRobot();
    }
    public double getRobotPositionX()
    {
        double distance=(distanceSensorX.getDistance(DistanceUnit.INCH));
        if (Double.isNaN(distance))
        {
            opMode.telemetry.log().add("DISTANCE Y VALUE WAS NaN");
            return 2.5;
        }
        if (distance>144)
        {
            opMode.telemetry.log().add("overshot X distance");
            return lastPosX;
        }
        lastPosX=distance;
        return distance;
    }
    public double getRobotPositionY()
    {
        double distance=distanceSensorY.getDistance(DistanceUnit.INCH);
        if (Double.isNaN(distance))
        {
            opMode.telemetry.log().add("DISTANCE Y VALUE WAS NaN");
            return 2.5;
        }
        if (distance>144)
        {
            opMode.telemetry.log().add("overshot Y distance");
            return lastPosY;
        }
        lastPosY=distance;
        return distance;
    }
    double getSlope(double x1, double y1, double x2, double y2)
    {
        return (y1-y2)/(x1-x2);
    }
}
