package org.firstinspires.ftc.teamcode.realsense;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.roadRunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadRunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadRunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;
import org.firstinspires.ftc.teamcode.util.state.Button;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.getMotorVelocityF;

/*
 * Simple mecanum drive hardware implementation for REV hardware. If your hardware configuration
 * satisfies the requirements, SampleMecanumDriveREVOptimized is highly recommended.
 */
public class DriveTrain6547Realsense extends DriveBase6547Realsense {

    private static double angleZzeroValue=0;

    ElapsedTime runtime = new ElapsedTime();

    public final String GYRO_ANGLE_FILE_NAME="gyroAngle.txt";
    public final String ROBOT_POS_FILE_NAME = "robotPos.txt";

    public Button a1,a2,b1,b2,x1,x2,y1,y2,dpadUp1,dpadUp2,dpadDown1, dpadDown2, dpadLeft1, dpadLeft2,dpadRight1, dpadRight2,leftBumper1,leftBumper2,rightBumper1,rightBumper2,start1,start2, rightTrigger1, rightTrigger2, leftTrigger1, leftTrigger2;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    List<LynxModule> allHubs;

    LinearOpMode opMode;
    HardwareMap hardwareMap;

    public DriveTrain6547Realsense(LinearOpMode _opMode) {
        super(_opMode.hardwareMap);

        opMode = _opMode;
        hardwareMap = opMode.hardwareMap;

      //  LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //get gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        RobotLog.d("Initialized IMU");

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

        setLocalizer(new T265LocalizerSelfContained(hardwareMap)); //realsense
        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap)); //three wheel odometry

        initOtherHardware();

    }
    public void initOtherHardware()
    {
        allHubs = hardwareMap.getAll(LynxModule.class);

        RobotLog.d("Initialized hardware");

        initGamepads();

        RobotLog.d("Initialized gamepads");

        setBulkReadAuto();

        if (Math.abs(getPoseEstimate().getHeading())>Math.toRadians(15))
        {
            RobotLog.setGlobalWarningMessage("REALSENSE ANGLE OFF, CLOSE AND REOPEN ROBOT APP TO RESET AMGLE");
        }

        //opMode.telemetry = dashboard.getopMode.telemetry();
    }
    private void initGamepads() //set the buttons to thier values
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
    public void updateGamepads() //update the gamepad buttons from HOMAR-FTC-Libary for tele-op
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
    @Deprecated
    public void startRealsense()
    {
//        RobotLog.v("staring realsense");
//        slamra.start();
    }
    @Deprecated
    public void stopRealsense()
    {
//        RobotLog.v("Stopping Realsense");
//        slamra.stop();
    }
    public void setBulkReadAuto()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void setBulkReadManual()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void clearBulkReadCache()
    {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
    public void setBulkReadOff()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }
    public void writeFile(String filename, double number)
    {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, Double.toString(number));
            opMode.telemetry.log().add("saved " + number + " in " + filename);
        }
        catch(Exception e)
        {
             opMode.telemetry.log().add("Unable to write " + number + " in " + filename);
        }
    }
    public void writeFile(String filename, String str)
    {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, str);
            opMode.telemetry.log().add("saved \"" + str + "\" in " + filename);
        }
        catch(Exception e)
        {
            opMode.telemetry.log().add("Unable to write \"" + str + "\" in " + filename);
        }
    }
    public double readFile(String filename)
    {
        try {
            double output=0;
            File file= AppUtil.getInstance().getSettingsFile(filename);
            output = Double.parseDouble(ReadWriteFile.readFile(file).trim());
            opMode.telemetry.log().add("read " + output + " in " + filename);
            return output;
        }
        catch (Exception e)
        {
            opMode.telemetry.log().add("Unable to read " + filename + ", returning 0");
            return 0;
        }
    }
    public String readFileString(String filename)
    {
        try {
            String output;
            File file= AppUtil.getInstance().getSettingsFile(filename);
            output = ReadWriteFile.readFile(file).trim();
            opMode.telemetry.log().add("read " + output + " in " + filename);
            return output;
        }
        catch (Exception e)
        {
            opMode.telemetry.log().add("Unable to read " + filename + ", returning 0");
            return "";
        }
    }
    public void saveRobotPos()
    {
        //Not using toString because that rounds the pos to 3 decimal places.  I want more decimal places because I want to
        Pose2d currentPos = getPoseEstimate();
        double x = currentPos.getX();
        double y = currentPos.getY();
        double angleRAD = currentPos.getHeading();
        StringBuilder stringBuilder = new StringBuilder();
        String pos = stringBuilder.append(x).append(",").append(y).append(",").append(angleRAD).toString();
        writeFile(ROBOT_POS_FILE_NAME,pos);
    }
    public Pose2d readRobotPos()
    {
        String posString = readFileString(ROBOT_POS_FILE_NAME);
        String[] numStrings = posString.split(",");

        double x = Double.parseDouble(numStrings[0]);
        double y = Double.parseDouble(numStrings[1]);
        double angleRAD = Double.parseDouble(numStrings[2]);

        return new Pose2d(x,y,angleRAD);
    }
    public void outputTelemetry() //used for debugging.  This method is called a lot
    {
        opMode.telemetry.addData("IMU angle with zero Value", getIMUAngle());
        opMode.telemetry.update();
    }
    //updates servo for use with a gamepad stick
    public void updateServo(Servo servo, double gamepadStick, double speed, double max, double min)
        {
            if (Math.abs(gamepadStick) < .2) return;
            double posToAdd = gamepadStick*speed;
            opMode.telemetry.addData("changing pos by ",posToAdd);
            double servoCurrentPos = servo.getPosition();
            double targetPos = posToAdd + servoCurrentPos;
            //if ((servoCurrentPos > min && gamepadStick > 0) || (servoCurrentPos < max && gamepadStick < 0))
            //{
            if (targetPos > max) servo.setPosition(max);
            else if (targetPos < min) servo.setPosition(min);
            else {
                servo.setPosition(targetPos);
            }
        //}
    }
    public void updateServo(Servo servo, double gamepadStick, double speed)
    {
        updateServo(servo, gamepadStick, speed, 0, 1);
    }
    @Override
    public void runAtAllTimes() //anything in here runs at all times during auton because this method is ran during roadRunner's state machine
    {

    }
    public double getIMUAngle() //gets the gyro angle in degrees
    {
        double currentAngle=Math.toDegrees(getRawExternalHeading())+angleZzeroValue;
        while (currentAngle>180) currentAngle-=360;
        while (currentAngle<-180) currentAngle+=360;
        return currentAngle;
    }
    public double getIMUAngle(boolean extendBeyond180)
    {
        double currentAngle=Math.toDegrees(getRawExternalHeading())+angleZzeroValue;
        if (extendBeyond180) return currentAngle;
        else return getIMUAngle();
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
    public void disableEncoders() //turn all encoders off
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public DcMotor zeroEncoder(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }
    public void stopRobot()
    {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    //--------------------------------------------------------------
    // Get Set Methods
    //--------------------------------------------------------------

    public double getAngleZzeroValue() {
        return angleZzeroValue;
    }

    public void setAngleZzeroValue(double angleZzeroValue) {
        DriveTrain6547Realsense.angleZzeroValue = angleZzeroValue;
    }
    public ElapsedTime getRuntime() {
        return runtime;
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
    public void setMotorPowers(double LF, double LR, double RR, double RF) {
        leftFront.setPower(LF);
        leftRear.setPower(LR);
        rightRear.setPower(RR);
        rightFront.setPower(RF);
    }
    //set motor powers but it's field realtive.  Unused
    public void setMotorPowers(double v,double v1, double v2, double v3, double angle)
    {
        angle-=getIMUAngle();
        angle+=45;
        angle = Math.toRadians(angle);
        double normalize = 1/Math.cos(Math.toRadians(45));
        v*=Math.cos(angle)*normalize;
        v1*=Math.cos(angle)*normalize;
        v2*=Math.sin(angle)*normalize;
        v3*=Math.sin(angle)*normalize;
        setMotorPowers(v,v1,v2,v3);
    }
    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    /*
    test road runner methods, currently not used
     */
    /*
    Road Runner turn with consideration of the gyro angle
     */
    public void turnRealtiveSync(double angle)
    {
        double target=angle-getPoseEstimate().getHeading();
        //target-=Math.toRadians(90);
        if (Math.abs(target)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
        {
            target+=(target>=0) ? Math.toRadians(-360) : Math.toRadians(360);
        }
        opMode.telemetry.log().add("inputted Angle: " + angle + " , turning to: " + target);
        RobotLog.d("Turning Realtive to heading " + angle + ", amount turning: " + target);
        turnSync(target);
    }
    public void turnRealtive(double angle)
    {
        double target=angle-Math.toRadians(getIMUAngle());
        target-=Math.toRadians(90);
        if (Math.abs(target)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
        {
            target+=(target>=0) ? Math.toRadians(-360) : Math.toRadians(360);
        }
        opMode.telemetry.log().add("inputted Angle: " + angle + " , turning to: " + target);
        turn(target);
    }
    public void updateRobotPosRoadRunner()
    {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        fieldOverlay.setStroke("#F44336");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        RobotLog.v("SENTING IT TO TELEmetry, x:" + currentPose.getX() + "y:" + currentPose.getY() + "R:" + currentPose.getHeading());

        dashboard.sendTelemetryPacket(packet);

    }
}
