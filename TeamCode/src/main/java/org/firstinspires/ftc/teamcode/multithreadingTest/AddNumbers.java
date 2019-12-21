package org.firstinspires.ftc.teamcode.MuiltthreadingTest;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AddNumbers implements Runnable {

    public boolean isRunning = true;

    public AddNumbers(DcMotor motor)
    {
        this.motor = motor;
    }
    public int encoderValue=0;
    public DcMotor motor;
    @Override
    public void run() {
        while (isRunning) encoderValue = motor.getCurrentPosition();
    }
    public void stop()
    {
        isRunning = false;
    }
    public int getEncoderValue() {
        return encoderValue;
    }
}
