//package org.firstinspires.ftc.teamcode.testing;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.oldPrograms.usedAtState.DriveTrain6547State;
//import org.firstinspires.ftc.teamcode.util.MiniPID;
//
//import java.util.WeakHashMap;
//import java.util.concurrent.TimeUnit;
//
//@Config
//@TeleOp
//public class PIDdriveTest extends LinearOpMode {
//
//    DriveTrain6547State bot;
//    public static double Y_P_COEFF = 0;//0.17825353626292278;
//    public static double Y_I_COEFF = 0;//0.2852056580206765;
//    public static double Y_D_COEFF = 0;//0.0003;
//    public static double X_P_COEFF = 0;//0.225;
//    public static double X_I_COEFF = 0;//0.37869858549483094;
//    public static double X_D_COEFF = 0;//0.00045;
//    public static double T_P_COEFF = 0;//0.0168;
//    public static double T_I_COEFF = 0;//0.0240;
//    public static double T_D_COEFF = 0;//0.00296;
//
//    public static double targetX = 0;
//    public static double targetY = 0;
//    public static double targetHeadingDegrees = 0;
//
//    public static double radius = 1;
//    public static double maxPower = .1;
//
//    private double maxRpm = DriveConstants.getMaxRpm();
//    public static double GEAR_RATIO = .5;
//    public static double WHEEL_RADIUS = 2;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        bot = new DriveTrain6547State(this);
//
//        telemetry.addData("Ready to start","");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            driveTo(new Pose2d(targetX,targetY,Math.toRadians(targetHeadingDegrees)), radius,maxPower);
//        }
//
//
//    }
//    public void driveTo(Pose2d pos, double radius) {driveTo(pos,radius,.1);}
//    public void driveTo(Pose2d pos, double radius, double powMaxAtTarget)
//    {
//        MiniPID xPID = new MiniPID(X_P_COEFF,X_I_COEFF,X_D_COEFF);
//        MiniPID yPID = new MiniPID(Y_P_COEFF,Y_I_COEFF,Y_D_COEFF);
//        MiniPID turnPID = new MiniPID(T_P_COEFF,T_I_COEFF,T_D_COEFF);
//
//        Pose2d ogPos = bot.getPoseEstimate();
//
//        xPID.setOutputLimits(0,1);
//        yPID.setOutputLimits(0,1);
//        turnPID.setOutputLimits(0,1);
//
//        xPID.setSetpoint(pos.getX());
//        yPID.setSetpoint(pos.getY());
//        turnPID.setSetpoint(pos.getHeading());
//
//        while ((distance(bot.getPoseEstimate(), pos) > radius || avgPow() > powMaxAtTarget) && opModeIsActive())
//        {
//            xPID.setP(X_P_COEFF);
//            xPID.setI(X_I_COEFF);
//            xPID.setD(X_D_COEFF);
//            yPID.setP(Y_P_COEFF);
//            yPID.setI(Y_I_COEFF);
//            yPID.setD(Y_D_COEFF);
//            turnPID.setP(T_P_COEFF);
//            turnPID.setI(T_I_COEFF);
//            turnPID.setD(T_D_COEFF);
//
//            bot.updateRobotPosRoadRunner();
//
//            Pose2d currentPos = bot.getPoseEstimate();
//            double outputX = xPID.getOutput(currentPos.getX());
//            double outputY = yPID.getOutput(currentPos.getY());
//            double outputTurn = turnPID.getOutput(currentPos.getHeading());
//
//            double targetAngle = Math.atan2(outputY,outputX) - currentPos.getHeading();
//            double speed = Math.hypot(outputX,outputY);
//
//            double leftFrontPower =  speed * Math.cos(targetAngle) + outputTurn;
//            double rightFrontPower =  speed * Math.sin(targetAngle) - outputTurn;
//            double leftBackPower =  speed * Math.sin(targetAngle) + outputTurn;
//            double rightBackPower =  speed * Math.cos(targetAngle) - outputTurn;
//
//            bot.setMotorPowers(leftFrontPower,leftBackPower,rightBackPower,rightFrontPower);
//        }
//        bot.stopRobot();
//
//    }
//    double avgPow()
//    {
//        double pow = bot.leftFront.getPower() + bot.rightFront.getPower() + bot.rightRear.getPower() + bot.leftRear.getPower();
//        return pow/4;
//    }
//    double distance(Pose2d p1,Pose2d p2)
//    {
//        double x = p1.getX() - p2.getX();
//        double y = p1.getY() - p2.getY();
//
//        double dist = Math.hypot(x,y);
//        return dist;
//    }
//    public static double rpmToVelocity(double rpm) {
//        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
//
//    }
//    private double getAngularMaxVelocity(){
//        return getAngularMaxVelocity(AngleUnit.DEGREES,TimeUnit.SECONDS)
//    }
//    private double getAngularMaxVelocity(AngleUnit angleUnit){
//        return getAngularMaxVelocity(angleUnit,TimeUnit.SECONDS);
//    }
//    private double getAngularMaxVelocity(AngleUnit angleUnit, TimeUnit timeUnit)
//    {
//        double revPerTimeUnit = maxRpm;
//        if (timeUnit == TimeUnit.SECONDS) {
//            revPerTimeUnit/=60;
//        } else if (timeUnit == TimeUnit.MINUTES) {} //do nothing, its fine how it is
//        else RobotLog.setGlobalWarningMessage("INVALID TIME UNIT, CAN ONLY BE SECONDS OR MINUTES");
//        //change rev per sec to Radians per sec
//        if (angleUnit == AngleUnit.RADIANS ) {
//            return revPerTimeUnit * 2 * Math.PI;
//        } else { //angle unit is equal to DEGREES
//            return revPerTimeUnit * 360;
//        }
//    }
//    private double linearToAngularVelocity(double vel, AngleUnit angleUnit) {
//        if (angleUnit == AngleUnit.DEGREES) {
//            vel = Math.toRadians(vel);
//        }
//        double raduis = WHEEL_RADIUS * GEAR_RATIO;
//        return 0;
//    }
//    private double velocityToRPM(AngleUnit angleUnit, TimeUnit timeUnit)
//    {
//
//    }
//}