package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePosition implements Runnable{
    //Odometry wheels
    private DcMotor verticalEncoder, horizontalEncoder;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double verticalEncoderWheelPosition, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition(DcMotor verticalEncoder, DcMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay){
        this.verticalEncoder = verticalEncoder;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        verticalEncoderWheelPosition = (verticalEncoder.getCurrentPosition() * verticalEncoderPositionMultiplier);

        double vertChange = verticalEncoderWheelPosition - previousVerticalEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = 0; //TODO: add IMU angle
        robotOrientationRadians = 0; //TODO: add IMU angle

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = vertChange;
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + (n*Math.cos(robotOrientationRadians)));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - (n*Math.sin(robotOrientationRadians)));

        previousVerticalEncoderWheelPosition = verticalEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseVerticalEncoder()
    {
        verticalEncoderPositionMultiplier *= -1;
    }

    public void reverseNormalEncoder(){
//        if(normalEncoderPositionMultiplier == 1){
//            normalEncoderPositionMultiplier = -1;
//        }else{
//            normalEncoderPositionMultiplier = 1;
//        }
        normalEncoderPositionMultiplier*=-1;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
