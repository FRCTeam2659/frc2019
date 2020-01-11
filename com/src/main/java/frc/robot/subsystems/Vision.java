package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * instantializing all member components at the start of the match.
 */
public class Vision extends Subsystem {
    private static Vision mInstance;

    private boolean currentVisionMode = false;
    private boolean m_LimelightHasValidTarget = false; 
    private double[] camtran;
    private double tv;
    private double tx;
    private double ty;
    private double tz;
    private double ta;

    public synchronized static Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
        }
        return mInstance;
    }

    private Vision() {
    }

    
    /**
     * @return true if the robot is receiving data from the scale tracker
     */
    public boolean isConnected() {
        return true;
    }

    public Pose2d getCoordinate() {
        Pose2d targetPose = new Pose2d(-camtran[2] - 20, camtran[0] - 9, Rotation2d.fromDegrees(-camtran[4]));;
        return targetPose;
    }

    public void changeCameraMode(boolean visionMode) {
        if (currentVisionMode != visionMode) {
            if (visionMode) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
                currentVisionMode = true;
            } else {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
                currentVisionMode = false;
            }
        }

    }

    public boolean isDataValid()
    {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        //tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("X").getDouble(0);
        //ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("Y").getDouble(0);
        //tz = NetworkTableInstance.getDefault().getTable("limelight").getEntry("Z").getDouble(0);
        //ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        //camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(camtran);

        if (tv < 1.0) {
          m_LimelightHasValidTarget = false;
          return false;
        }
        m_LimelightHasValidTarget = true;
        return true;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        isDataValid();
        SmartDashboard.putBoolean("Connected to Vision", isConnected());
        //SmartDashboard.putNumber("X", camtran[0]);
        //SmartDashboard.putNumber("Y", camtran[1]);
        //SmartDashboard.putNumber("Z", camtran[2]);
        //SmartDashboard.putNumber("Theta", camtran[4]);
    }

    @Override
    public void stop() {
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
}