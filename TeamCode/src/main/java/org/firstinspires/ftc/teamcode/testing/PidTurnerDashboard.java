package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;
import org.firstinspires.ftc.teamcode.util.MiniPID;

@Config
@Disabled
@TeleOp
public class PidTurnerDashboard extends SkyStone6547Meet2 {

    public static boolean TURN = true;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double targetAngle = 90;
    public static double turnTime = 3;
    public static double SpeedModifer=1;

    @Override
    public void runOpMode() {
        INIT(hardwareMap);
        initIMU();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        MiniPID miniPID = new MiniPID(P,I,D);

        telemetry.log().add("ready to start");

        waitForStart();

        while (opModeIsActive())
        {
            TurnPID(targetAngle,turnTime,SpeedModifer,miniPID);
            TURN=false;
            while (!TURN && opModeIsActive())
            {
                miniPID.setP(P);
                miniPID.setI(I);
                miniPID.setD(D);
                telemetry.addData("P", miniPID.getP());
                telemetry.addData("I", miniPID.getI());
                telemetry.addData("D", miniPID.getD());
                outputTelemetry();
            }
            telemetry.log().add("TURN IS TRUE");

        }
    }
}
