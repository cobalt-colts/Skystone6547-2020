//package org.firstinspires.ftc.teamcode.realsense;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.localization.Localizer;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Transform2d;
//import com.arcrobotics.ftclib.geometry.Translation2d;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.RobotLog;
//import com.spartronics4915.lib.T265Camera;
//
//import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveTrain6547Offseason;
//import org.jetbrains.annotations.NotNull;
//import org.jetbrains.annotations.Nullable;
//
//public class T265Localizer implements Localizer {
//
//   private DriveTrain6547Offseason bot;
//    private Pose2d poseOffset;
//    private Pose2d mPoseEstimate;
//    private Pose2d rawPose;
//    private T265Camera.CameraUpdate up;
//
//    public T265Localizer(DriveTrain6547Offseason bot) {
//        this.bot = bot;
//    }
//
//    @NotNull
//    @Override
//    public Pose2d getPoseEstimate() {
//        up = DriveTrain6547Offseason.slamra.getLastReceivedCameraUpdate();
//        //convert FTClib Pose2d to AMCE robotices Pose2D
//        //TODO: convert all FTClib geometry to ACME robotics geometry in T265Camera.java
//        try {
//            Translation2d oldPose = up.pose.getTranslation();
//            rawPose = new Pose2d(oldPose.getX(), oldPose.getY(), bot.getRawExternalHeading());
//            mPoseEstimate = rawPose.plus(poseOffset); //offsets the pose to be what the pose estimate is.
//            return mPoseEstimate;
//        } catch (Exception e)
//        {
//            RobotLog.v("NULLL object");
//            return new Pose2d(0,0,0);
//        }
//    }
//
//    //doesn't accully set the pose, it creates an offset Pose that the program adds to the camera pos to get to the pose estimate
//    @Override
//    public void setPoseEstimate(@NotNull Pose2d pose2d) {
//        poseOffset = pose2d.minus(rawPose);
//    }
//
//    @Override
//    public void update() {
//
//    }
//}
