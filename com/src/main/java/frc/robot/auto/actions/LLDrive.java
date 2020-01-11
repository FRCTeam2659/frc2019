package frc.robot.auto.actions;

import frc.lib.util.TortoDriveHelper;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LLDrive implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final TortoDriveHelper mTortoDriveHelper = new TortoDriveHelper();

    private double mStartTime;
    final double STEER_K = 0.08;                    // how hard to turn toward the target
    final double DRIVE_K = 0.2;                    // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 5.6;        // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.5;  
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    public LLDrive() {
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        Vision.getInstance().changeCameraMode(true);
    }

    @Override
    public void update() {
                         // Simple speed limit so we don't drive too fast

        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        //ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);


        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
        mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(0.4, m_LimelightSteerCommand, true, true));
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = Timer.getFPGATimestamp() - mStartTime > 1.6;
        return timedOut;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(0, 0, true, true));
        Vision.getInstance().changeCameraMode(false);
    }
}