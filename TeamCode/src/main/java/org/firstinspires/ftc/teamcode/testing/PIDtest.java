package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.MiniPID;
import org.firstinspires.ftc.teamcode.oldPrograms.usedInMeet2.SkyStone6547Meet2;

import org.firstinspires.ftc.teamcode.util.state.Button;
import org.firstinspires.ftc.teamcode.util.state.ToggleInt;
import org.firstinspires.ftc.teamcode.util.state.ToggleBoolean;

@TeleOp(name="PID tuner")
@Disabled
public class PIDtest extends SkyStone6547Meet2 {

    static final private double ROTATION_POWER = 0.5;
    
    private double stepP = 1, stepI = 0.01, stepD = 0.01;
    private Button up = new Button();
    private Button down = new Button();
    private Button left = new Button();
    private Button right = new Button();
    private Button y = new Button();
    private Button b = new Button();
    private ToggleBoolean a = new ToggleBoolean();
    private ToggleInt selected = new ToggleInt(3);

    MiniPID miniPID = new MiniPID(0.004,0,0.0036);

    double targetAngle=90;
    
    double actual;
    double output;

    @Override
    public void runOpMode() {
        INIT(hardwareMap);
        initIMU();
        
        telemetry.log().add("ready to start");
        waitForStart();

        while (opModeIsActive())
        {
            up.input(gamepad1.dpad_up);
            down.input(gamepad1.dpad_down);
            left.input(gamepad1.dpad_left);
            right.input(gamepad1.dpad_right);
            y.input(gamepad1.y);
            selected.input(gamepad1.x);
            a.input(gamepad1.a);
            b.input(gamepad1.b);

            if (up.onPress()) {
                switch (selected.output()) {
                    case 0:
                        miniPID.setP(miniPID.getP() + stepP);
                        break;
                    case 1:
                        miniPID.setI(miniPID.getI()+stepI);
                        break;
                    case 2:
                        miniPID.setD(miniPID.getD()+stepD);
                        break;
                }
            }
            if (down.onPress()) {
                switch (selected.output()) {
                    case 0:
                        miniPID.setP(miniPID.getP()-stepP);
                        break;
                    case 1:
                        miniPID.setI(miniPID.getI()-stepI);
                        break;
                    case 2:
                        miniPID.setD(miniPID.getD()-stepD);
                        break;
                }
            }
            if (left.onPress()) {
                switch (selected.output()) {
                    case 0:
                        stepP *= 10;
                        break;
                    case 1:
                        stepI *= 10;
                        break;
                    case 2:
                        stepD *= 10;
                        break;
                }
            }
            if (right.onPress()) {
                switch (selected.output()) {
                    case 0:
                        stepP /= 10;
                        break;
                    case 1:
                        stepI /= 10;
                        break;
                    case 2:
                        stepD /= 10;
                        break;
                }
            }

            if (y.onPress()) miniPID.reset();

            if (b.onPress())
            {
                angleZzeroValue=0;
                angleZzeroValue=-getIMUAngle();
            }

            if (a.output()) {
                telemetry.addData("Status", "Paused");
                turnLeft(0);
            }
            else {
                telemetry.addData("Status", "Running");
                if (gamepad1.left_bumper && gamepad1.right_bumper) turnLeft(0);
                else if (gamepad1.left_bumper) turnLeft(ROTATION_POWER);
                else if (gamepad1.right_bumper) turnLeft(-ROTATION_POWER);
                else TurnPIDForever(targetAngle);
            }

            telemetry.addData("Controls", "X to select, Y to reset, A to pause");
            String[] KCaptions = new String[] {"KP", "KI", "KD"};
            KCaptions[selected.output()] = "->"+KCaptions[selected.output()];
            String[] stepCaptions = new String[] {"stepP", "stepI", "stepD"};
            stepCaptions[selected.output()] = "->"+stepCaptions[selected.output()];
            telemetry.addData(KCaptions[0], miniPID.getP());
            telemetry.addData(KCaptions[1], miniPID.getI());
            telemetry.addData(KCaptions[2], miniPID.getD());
            telemetry.addData(stepCaptions[0], stepP);
            telemetry.addData(stepCaptions[1], stepI);
            telemetry.addData(stepCaptions[2], stepD);
            telemetry.update();
        }
    }

    public void TurnPIDForever(double angle) //use PID to turn the robot
    {
//        double angleDifference=angle-getIMUAngle();
//        if (Math.abs(angleDifference)>180) //make the angle difference less then 180 to remove unnecessary turning
//        {
//            angleDifference+=(angleDifference>=0) ? -360 : 360;
//        }
//        double tempZeroValue=angleZzeroValue;

//        miniPID.setOutputLimits(1);
//
//        miniPID.setSetpointRange(40);

//        double actual=0;
//        double output=0;
//        telemetry.log().add("angle difference " + angleDifference);
          //double target=angleDifference;
//        miniPID.setSetpoint(0);
//        miniPID.setSetpoint(target);
//        telemetry.log().add("target: " + target + "degrees");
//        runtime.reset();

            output = miniPID.getOutput(actual, targetAngle);
            //if (angle>175 || angle <-175) actual = getIMUAngle(true);
            actual = getIMUAngle();
            turnLeft(output);
            telemetry.addData("power", LeftFront.getPower());
            telemetry.addData("angle", actual);
            telemetry.addData("target angle", targetAngle);
            //outputTelemetry();
    }
}
