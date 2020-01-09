package org.firstinspires.ftc.teamcode.testing.MuiltthreadTest.multithreadingTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

@Config
@TeleOp
@Disabled
public class TestMuiltiThread extends SkyStone6547Meet2 {

    public static double SQUARE_SIZE=18;
    public static int STROKE_WIDTH = 1;
    public static double POS_MODIFER = 1;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        INIT(hardwareMap);

        initIMU();

        telemetry.log().add("ready to start");
        //AddNumbers encoderValue = new AddNumbers(LeftFront);
        //Thread encoderThread = new Thread(encoderValue);
        CalcPos pos = new CalcPos(new DcMotor[] {LeftBack, LeftFront, RightBack, RightFront});
        Thread posThread = new Thread(pos);
        waitForStart();
        posThread.start();
        //encoderThread.start();



        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStroke("black")
                    .setStrokeWidth(STROKE_WIDTH)
                    .strokeRect(pos.getXDist()*POS_MODIFER, pos.getYDist()*POS_MODIFER, SQUARE_SIZE, SQUARE_SIZE);
                    //.strokeRect(0,encoderValue.getEncoderValue()*POS_MODIFER, SQUARE_SIZE, SQUARE_SIZE);

            //telemetry.addData("Encoder value thread", encoderValue.getEncoderValue());
            //telemetry.addData("Encoder Value real", LeftFront.getCurrentPosition());
            //telemetry.update();
            dashboard.sendTelemetryPacket(packet);
            //telemetry multipleTelemetry = new MultipleTelemetry();
            telemetry.addData("X Pos", pos.getXDist());
            telemetry.addData("Y Pos", pos.getYDist());
            //telemetry.addData("Angle", pos.getAngle());
            telemetry.update();

        }
        pos.stop();
        //encoderValue.stop();


    }
}
