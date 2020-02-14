package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "Raw Odometry Test", group = "test")
public class rawOdometryTest extends LinearOpMode {

    private DcMotor sideEncoder,rightFrontEncoder,leftFrontEncoder;

    final double COUNTS_PER_INCH = 1743.855179349648;

    double leftVertIn, rightVertIn, sideIn;

    List<LynxModule> allHubs;

    @Override
    public void runOpMode() throws InterruptedException {
        rightFrontEncoder = hardwareMap.dcMotor.get("sideEncoder");
        sideEncoder = hardwareMap.dcMotor.get("vertRight");
        leftFrontEncoder = hardwareMap.dcMotor.get("intake");

        zeroEncoders();
        //allHubs = hardwareMap.getAll(LynxModule.class);
        telemetry.log().add("ready to start");
        waitForStart();

        //setBulkReadAuto();

        while (opModeIsActive())
        {
            leftVertIn = leftFrontEncoder.getCurrentPosition()/COUNTS_PER_INCH;
            rightVertIn = rightFrontEncoder.getCurrentPosition()/COUNTS_PER_INCH;
            sideIn = sideEncoder.getCurrentPosition()/COUNTS_PER_INCH;
            if (gamepad1.a)
            {
                zeroEncoders();
            }
            telemetry.addData("LEFT in.", leftVertIn);
            telemetry.addData("RIGHT in." ,rightVertIn);
            telemetry.addData("SIDE in.", sideIn);
            telemetry.addData("left Vert Encoder Pos",leftFrontEncoder.getCurrentPosition());
            telemetry.addData("right Vert Encoder Pos",rightFrontEncoder.getCurrentPosition());
            telemetry.addData("side Encoder Pos", sideEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
    public void zeroEncoders()
    {
        sideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sideEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setBulkReadAuto()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
