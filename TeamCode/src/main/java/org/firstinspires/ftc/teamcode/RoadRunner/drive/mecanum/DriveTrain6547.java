package org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
import org.firstinspires.ftc.teamcode.RoadRunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.RoadRunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.SkyStoneLoc;
import org.firstinspires.ftc.teamcode.util.MiniPID;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutionException;

import edu.spa.ftclib.internal.state.Button;

import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.getMotorVelocityF;

/*
 * Simple mecanum drive hardware implementation for REV hardware. If your hardware configuration
 * satisfies the requirements, SampleMecanumDriveREVOptimized is highly recommended.
 */
public class DriveTrain6547 extends SampleMecanumDriveBase {
    final double encoderTicksPerRotation=205;
    final double circumferenceOfWheel=12.566370614359172;

    public static double angleZzeroValue=0;

    ElapsedTime runtime = new ElapsedTime();

    public final String GYRO_ANGLE_FILE_NAME="gyroAngle.json";

    double deltaX=0;
    double deltaY=0;
    double oldDist=0;

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

    double lastPosX;
    double lastPosY;

    Rev2mDistanceSensor distanceSensorX;
    Rev2mDistanceSensor distanceSensorY;

    List<Integer> screamIds = new ArrayList<>();

    ElapsedTime screamTime = new ElapsedTime();

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    public Orientation angles;
    Acceleration gravity;
    Velocity thing;

    OpMode opMode;
    HardwareMap hardwareMap;

    public DriveTrain6547(OpMode _opMode) {

        opMode = _opMode;
        hardwareMap = opMode.hardwareMap;

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        initOtherHardware();

    }
    public void initOtherHardware()
    {
        distanceSensorX = hardwareMap.get(Rev2mDistanceSensor.class, "d boi");
        distanceSensorY = hardwareMap.get(Rev2mDistanceSensor.class, "d boi1");
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

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        opMode.telemetry.log().add("set hardware");

        setFondationGrabber(0);

        opMode.telemetry.log().add("Initialized IMU");

        initGamepads();

        opMode.telemetry.log().add("Initialized gamepads");

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
    public void initIMUTelemetry() {
        opMode.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                thing = imu.getVelocity();
            }
        });
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
//        opMode.telemetry.addData("Front Left POW", leftFront.getPower());
//        opMode.telemetry.addData("Front Right POW", rightFront.getPower());
//        opMode.telemetry.addData("Back Left POW", leftRear.getPower());
//        opMode.telemetry.addData("Back Right POW", rightRear.getPower());
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
        double max2 = max+.14;
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
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
            opMode.telemetry.addData("power", leftFront.getPower());
            opMode.telemetry.addData("angle", actual);
            opMode.telemetry.update();
            outputTelemetry();
        }
        stopRobot();
        angleZzeroValue=tempZeroValue;

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        if (drivingDistanceInEncoderTicks>=0) while (rightRear.getCurrentPosition() <=drivingDistanceInEncoderTicks && ((LinearOpMode) opMode).opModeIsActive())
        {
            opMode.telemetry.addData("Status","Driving For Length");
            opMode.telemetry.addData("Encoder ticks to drive (positve)", drivingDistanceInEncoderTicks);
            opMode.telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        else while (rightRear.getCurrentPosition() >=drivingDistanceInEncoderTicks && ((LinearOpMode) opMode).opModeIsActive())
        {
            opMode.telemetry.addData("Status","Driving For Length");
            opMode.telemetry.addData("Encoder ticks to drive (negitive)", drivingDistanceInEncoderTicks);
            opMode.telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        stopRobot();
        opMode.telemetry.log().add("Done with Drive for length");
        opMode.telemetry.log().add("Left Front Encoder Pos:"+ leftFront.getCurrentPosition());
        opMode.telemetry.log().add("Right Front Encoder Pos:"+ rightFront.getCurrentPosition());
        opMode.telemetry.log().add("Left Back Encoder Pos:"+ leftRear.getCurrentPosition());
        opMode.telemetry.log().add("Right Back Encoder Pos:"+ rightRear.getCurrentPosition());
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
            leftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            rightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            leftRear.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            rightRear.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
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
            leftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            rightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            leftRear.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            rightRear.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            opMode.telemetry.addData("Status","Driving Field Relative");
            outputTelemetry();
        }
        if (!brake) //let the motors coast when the robot is stoped
        {
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        stopRobot();
        if (!brake) //revert the changes.
        {
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        leftFront.setPower(power * Math.cos(angleInRadians-robotAngle) + rightX);
        rightFront.setPower(power * Math.sin(angleInRadians-robotAngle) - rightX);
        leftRear.setPower(power * Math.sin(angleInRadians-robotAngle) + rightX);
        rightRear.setPower(power * Math.cos(angleInRadians-robotAngle) - rightX);

    }
    public void zeroEncoders() //set all the encoders to zero
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public DcMotor zeroEncoder(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
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
        leftFront.setPower(leftPow);
        rightFront.setPower(rightPow);
        leftRear.setPower(leftPow);
        rightRear.setPower(rightPow);
    }
    public void driveForward(double power)
    {
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(-power);
    }
    public void turnLeft(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }
    public void DriveLeft(double power) { //strafe left
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }
    public void stopRobot()
    {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
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
    public double averageDrivetrainEncoder() //average the position from the drive train motors OLD
    {
        double motorPosition=0;
        motorPosition+=Math.abs(leftRear.getCurrentPosition());
        motorPosition+=Math.abs(leftFront.getCurrentPosition());
        motorPosition+=Math.abs(rightRear.getCurrentPosition());
        motorPosition+=Math.abs(rightFront.getCurrentPosition());
        if (Double.isNaN(motorPosition/4)) return 0;
        else return Math.abs(motorPosition/4);
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


    //--------------------------------------------------------------
    //  Road Runner
    //--------------------------------------------------------------

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
