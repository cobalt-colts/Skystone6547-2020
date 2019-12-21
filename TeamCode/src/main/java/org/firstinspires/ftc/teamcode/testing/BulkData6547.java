package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

public class BulkData6547 {

    public RevBulkData bulkDataHub2;
    public RevBulkData bulkDataHub3;

    public BulkData6547(Object[] parts)
    {
        for (Object part : parts)
        {
            if (part.getClass().isAssignableFrom(DcMotor.class))
            {
                DcMotor tempMotor = (DcMotor) part;
            }
        }
    }

//    public int getExpantionHub(DcMotor motor)
//    {
//        String connectionInfo[] = motor.getConnectionInfo().split(";",2);
//        String module = connectionInfo[1];
//    }

    public void updateBulkData()
    {

    }
}
class revHub
{
    AnalogInput[] analogInputs = new AnalogInput[4];
    DigitalChannel[] digitalChannels= new DigitalChannel[8];
    ExpansionHubMotor[] expansionHubMotors = new ExpansionHubMotor[4];
    ExpansionHubEx expansionHub;
    ExpansionHubServo[] expansionHubServos= new ExpansionHubServo[6];

}