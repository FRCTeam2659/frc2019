package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotState;
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

    public synchronized static Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
        }
        return mInstance;
    }

    public enum ScaleHeight {
        LOW,
        NEUTRAL,
        HIGH
    }

    private Vision() {
    }

    private double dx;
    private double dy;
    private double dtheta;
    private double dpx;
    private double realDx;
    private double realDy;
    private double realDz;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");

    /**
     * @return true if the robot is receiving data from the scale tracker
     */
    public synchronized boolean isConnected() {
        return true;
    }

    public synchronized Pose2d getCoordinate() {
        NetworkTableEntry xEntry = table.getEntry("centerX");
        NetworkTableEntry targetWidth = table.getEntry("targetWidth");

        double gyroAngle = Drive.getInstance().getHeading().getDegrees();
        double centerX = xEntry.getDouble(0);
        dpx = Constants.kCenterScreenWidth - centerX;
        dtheta = Math.atan((centerX - Constants.kCenterScreenWidth)/Constants.H_FOCAL_LENGTH);
        dx = dpx * Constants.kVisionTapeWidth/targetWidth.getDouble(0);
        if (centerX > Constants.kCenterScreenWidth)
            gyroAngle = -gyroAngle;
        realDz = Math.sin(Math.toRadians(90+gyroAngle))*-dx/Math.sin(Math.abs(dtheta)) * Math.cos(Math.toRadians(gyroAngle));
        SmartDashboard.putNumber("thetaGplus90", 90+gyroAngle);
        //dy = dx / Math.tan(dtheta);
        dtheta = -dtheta;
        realDx = realDz * Math.sin(Math.toRadians(Math.toDegrees(dtheta)+gyroAngle));
        realDy = realDz * Math.cos(Math.toRadians(Math.toDegrees(dtheta)+gyroAngle));
        //Pose2d kSideStartPose = new Pose2d(Math.abs(dy)-13, dx, Drive.getInstance().getHeading()); // 13 camera offset Rotation2d.fromDegrees(dtheta)
        Pose2d currentPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        Pose2d targetPose;
        //if (gyroAngle < 180 && gyroAngle > 0)
            targetPose = new Pose2d(Math.abs(realDy + currentPose.getTranslation().y())-13*Math.sin(Math.toRadians(gyroAngle)), realDx + currentPose.getTranslation().x() - 13*Math.cos(Math.toRadians(gyroAngle)), Rotation2d.fromDegrees(0.0));
        //else
            //targetPose = new Pose2d(realDx + currentPose.getTranslation().x(), realDy + currentPose.getTranslation().y(), Rotation2d.fromDegrees(-90.0));
        return targetPose;
    }

    public synchronized double getDeltaX() {
        return dx;
    }


    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Connected to Vision", isConnected());
        SmartDashboard.putNumber("X", dx);
        SmartDashboard.putNumber("Y", dy);
        SmartDashboard.putNumber("realDX", realDx);
        SmartDashboard.putNumber("realDY", realDy);
        SmartDashboard.putNumber("realDZ", realDz);
        SmartDashboard.putNumber("Theta", dtheta);
        SmartDashboard.putNumber("dpx", dpx);
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
                /*synchronized (Vision.this) {
                    mAngle = SmartDashboard.getNumber("scaleAngle", Double.NaN);
                    mTip = SmartDashboard.getNumber("scaleTip", Double.NaN);
                    mError = SmartDashboard.getBoolean("scaleError", true);
                    double heartbeat = SmartDashboard.getNumber("scaleHeartbeat", -2);
                    if (heartbeat > mLastHeartbeatValue) {
                        mLastHeartbeatValue = heartbeat;
                        mLastHeartbeatTime = timestamp;
                    }
                }*/
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };

        looper.register(loop);
    }
}