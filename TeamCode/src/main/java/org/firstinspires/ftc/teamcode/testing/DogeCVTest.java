//package org.firstinspires.ftc.teamcode.testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//import java.util.Locale;
//
//@TeleOp
//@Disabled
//public class DogeCVTest extends LinearOpMode {
//
//    private OpenCvCamera phoneCam;
//    private SkystoneDetector skystoneDetector;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        int cameraViewMoniterId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
//        telemetry.log().add("got camera id");
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraViewMoniterId);
//        telemetry.log().add("got camera");
//
//        phoneCam.openCameraDevice();
//        telemetry.log().add("opened camera");
//
//        skystoneDetector = new SkystoneDetector();
//        telemetry.log().add("got Stone Detector");
//        phoneCam.setPipeline(skystoneDetector);
//        telemetry.log().add("put the Stone Detector in the Camera");
//
//        phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//        telemetry.log().add("Started streaming the camera");
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetry.log().add("ready to start");
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            telemetry.addData("Stone Pos X",skystoneDetector.getScreenPosition().x);
//            telemetry.addData("Stone Pos Y", skystoneDetector.getScreenPosition().y);
//            telemetry.addData("Frame Count",phoneCam.getFrameCount());
//            telemetry.addData("FPS",String.format(Locale.US, "$.2f",phoneCam.getFps()));
//            telemetry.addData("Total Frame Time ms", phoneCam.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline Time ms", phoneCam.getPipelineTimeMs());
//            telemetry.addData("Overhead Time ms",phoneCam.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS",phoneCam.getCurrentPipelineMaxFps());
//            telemetry.update();
//
//            if (gamepad1.a)
//            {
//                phoneCam.stopStreaming();
//            }
//            else if (gamepad1.x)
//            {
//                phoneCam.pauseViewport();
//            }
//            else if (gamepad1.y)
//            {
//                phoneCam.resumeViewport();
//            }
//        }
//    }
//}
