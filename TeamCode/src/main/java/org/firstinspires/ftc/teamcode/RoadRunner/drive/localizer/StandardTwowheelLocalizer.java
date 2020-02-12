package org.firstinspires.ftc.teamcode.RoadRunner.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.DriveTrain6547;

import java.util.Arrays;
import java.util.List;


/*
    Two Wheel Odometry tracking.
 */
@Config
public class StandardTwowheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192; //(8192 Counts per rev) 2048 Cycles per Revolution.  Info got from rev site
    public static double WHEEL_RADIUS = 0.8418087058333801; // in  //19 mm
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    public static final double COUNTS_PER_INCH = 1743.855179349648;

    private DcMotor frontEncoder,sideEncoder;

    private DriveTrain6547 bot;

    private List<LynxModule> allHubs;
    public StandardTwowheelLocalizer(HardwareMap hardwareMap, DriveTrain6547 bot) {
        super(Arrays.asList(
                new Pose2d(-5, 1.625, 0), //sideEncoder parrel to drive train
                new Pose2d(-5.125, -2, Math.toRadians(90)) // frontEncoder purpendiculat to drive train
        ));

        allHubs = hardwareMap.getAll(LynxModule.class);
        this.bot = bot;

        sideEncoder = hardwareMap.dcMotor.get("sideEncoder");
        frontEncoder = hardwareMap.dcMotor.get("rightFront");

        sideEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        setBulkReadManual();
    }

//    public static double encoderTicksToInches(int ticks) {
//        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
//    }
    public static double encoderTicksToInches(int ticks)
    {
        return ticks/COUNTS_PER_INCH;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        clearBulkReadCache();
        return Arrays.asList(
                encoderTicksToInches(sideEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @Override
    public double getHeading() {
        return bot.getRawExternalHeading();
    }

    private void setBulkReadManual()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    private void clearBulkReadCache()
    {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}
