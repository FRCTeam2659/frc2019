package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.Superstructure.SuperstructureStates;

public class Limelight extends Subsystem {
    private static Limelight mInstance;
    private NetworkTable mMainLimelight = NetworkTableInstance.getDefault().getTable("limelight-main");
    private NetworkTable mSecondaryLimelight = NetworkTableInstance.getDefault().getTable("limelight-second");
 
    private boolean mainCameraAsVision = true;
    private boolean currentVisionMode = false;
    private double[] camtran;

    public synchronized static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    private Limelight() {
    }

    
    @Override
    public void registerEnabledLoops(ILooper looper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };

        looper.register(loop);
    }

    public Pose2d getCoordinate() {
        Pose2d targetPose = new Pose2d(-camtran[2] - 20, camtran[0] - 9, Rotation2d.fromDegrees(-camtran[4]));;
        return targetPose;
    }

    public void useMainCameraAsVision() {
        mMainLimelight.getEntry("ledMode").setNumber(3);
        mMainLimelight.getEntry("camMode").setNumber(0);
        //mSecondaryLimelight.getEntry("ledMode").setNumber(1);
        //mSecondaryLimelight.getEntry("camMode").setNumber(1);
        mainCameraAsVision = true;
    }
    public void useSecondaryCameraAsVision() {
        mMainLimelight.getEntry("ledMode").setNumber(1);
        mMainLimelight.getEntry("camMode").setNumber(1);
        //mSecondaryLimelight.getEntry("ledMode").setNumber(3);
        //mSecondaryLimelight.getEntry("camMode").setNumber(0);
        mainCameraAsVision = false;
    }

    public boolean getLimelightMode() {
        return mainCameraAsVision;
    }

    public double getTargetX() {
        if (mainCameraAsVision)
            return mMainLimelight.getEntry("tx").getDouble(0);
        else
            return mSecondaryLimelight.getEntry("tx").getDouble(0);
    }

    public boolean getTargetExists() {
        if (mainCameraAsVision)
            return mMainLimelight.getEntry("tv").getDouble(0) == 1;
        else
            return mSecondaryLimelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        if (mainCameraAsVision)
            return mMainLimelight.getEntry("ta").getDouble(0);
        else
            return mSecondaryLimelight.getEntry("ta").getDouble(0);
    }

    public double getDesiredTargetArea() {
        if (mainCameraAsVision)
            return 7.9;
        else
            return 18.7;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("mainX", mMainLimelight.getEntry("tx").getDouble(0));
        SmartDashboard.putNumber("viceX", mSecondaryLimelight.getEntry("tx").getDouble(0));
    }

    @Override
    public void stop() {
    }
}