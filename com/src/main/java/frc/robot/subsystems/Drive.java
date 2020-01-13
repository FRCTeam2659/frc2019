package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.planners.DriveMotionPlanner;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.util.DriveSignal;
import frc.lib.util.PIDConstants;
import frc.lib.util.PIDWrapper;
import frc.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {

    private static final int kVelocityControlSlot = 0;
    private static final int kPositionControlSlot = 1;
    private static final double DRIVE_ENCODER_PPR = 4096.;
    private static Drive mInstance = new Drive();
    private Limelight mLimelight = Limelight.getInstance();
    
    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
    private final Solenoid mShifter;
    // Control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private double mLeftDemand;
    private double mRightDemand;
    private PIDWrapper straightPID, straightPID2;
	private PIDWrapper turnPID;
	private PIDWrapper limelightTurnPID;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    public boolean mOverrideTrajectory = false;
    private double[] ypr = new double[3];
    private double yOutput = 0;
    private double xOutput = 0;
    private boolean withinAngle = false;
    private boolean withinRange = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            stopLogging();
        }
    };

    private void configureMaster(TalonSRX talon, boolean left) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        talon.setInverted(left);
        talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
        configureMaster(mLeftMaster, true);
        mLeftMaster.setSensorPhase(false);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveAId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveA.setInverted(true);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveBId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveB.setInverted(true);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        configureMaster(mRightMaster, false);
        mRightMaster.setSensorPhase(false);
        
        mRightSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveAId,
                Constants.kRightDriveMasterId);
        mRightSlaveA.setInverted(false);

        mRightSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveBId,
                Constants.kRightDriveMasterId);
        mRightSlaveB.setInverted(false);

        mShifter = new Solenoid(Constants.kShifterSolenoidId);

        reloadGains();

        mPigeon = new PigeonIMU(mRightSlaveA);
        mRightSlaveA.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);
        mPigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);

        straightPID = new PIDWrapper(new PIDConstants(0.07, 0.0, 1.0, 0.5));// 4th eps
        straightPID.setMinDoneCycles(10);
        straightPID.setIRange(1);
        straightPID2 = new PIDWrapper(new PIDConstants(0.035, 0.0, 0.5, 0.5));
        straightPID2.setMinDoneCycles(10);
        limelightTurnPID = new PIDWrapper(new PIDConstants(0.04, 0.000, 0.4, 1));
        limelightTurnPID.setMinDoneCycles(10);
        limelightTurnPID.setMaxOutput(0.4);

        // Force a solenoid message.
        mIsHighGear = false;
        setHighGear(true);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();
    }

    public static Drive getInstance() {
        return mInstance;
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesToTicks(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI) * 4096;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);

            System.out.println("Switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mLeftMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setPosition(double left, double right) {
    		if (mDriveControlState != DriveControlState.POSITION) {
    			mLeftMaster.selectProfileSlot(kPositionControlSlot, 0);
                mRightMaster.selectProfileSlot(kPositionControlSlot, 0);
                setBrakeMode(true);
                mDriveControlState = DriveControlState.POSITION;
    		}
    		mLeftDemand = inchesToTicks(left)+mLeftMaster.getSelectedSensorPosition(0);
    		mRightDemand = inchesToTicks(right)+mRightMaster.getSelectedSensorPosition(0);
    		mLeftMaster.set(ControlMode.Position, mLeftDemand);
    		mRightMaster.set(ControlMode.Position, mRightDemand);
    }
    
    public synchronized boolean isDoneWithPosition() {
    		if (Math.abs(mLeftDemand - mLeftMaster.getSelectedSensorPosition(0)) < 500 && Math.abs(mRightDemand - mRightMaster.getSelectedSensorPosition(0)) < 500)
    			return true;
    		else
    			return false;
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public void driveLimeLightXY(double maxOutput) {
        if (mLimelight.getLimelightMode()) {
            straightPID.setMaxOutput(0.5);
            straightPID.setFinishedRange(0.5);
            straightPID.setMinDoneCycles(1);
            straightPID.setDesiredValue(mLimelight.getDesiredTargetArea());
            driveTurnToAngleWithForwardVelocity(straightPID.calcPID(mLimelight.getTargetArea())); //120 is max velocity
        } else {
            straightPID2.setMaxOutput(0.36);
            straightPID2.setFinishedRange(0.5);
            straightPID2.setMinDoneCycles(1);
            straightPID2.setDesiredValue(mLimelight.getDesiredTargetArea());
            driveTurnToAngleWithForwardVelocity(straightPID2.calcPID(mLimelight.getTargetArea())); //120 is max velocity
        }
    }
    
	public void driveLimeLightXY() {
		driveLimeLightXY(1.0);
	}

	public void driveTurnToAngleWithForwardVelocity(double yOutput) {
		double eps = 1.0;
		double theta = mLimelight.getTargetX() + angle;
		double maxTurn = 1;
		this.yOutput = yOutput;
		limelightTurnPID.setFinishedRange(eps);
		imelightTurnPID.setDesiredValue(theta);

		double x = limelightTurnPID.calcPID(angle);
		if (x > maxTurn)
			x = maxTurn;
        else if (x < -maxTurn)
			x = -maxTurn;

		if (Math.abs(getHeadingDegree() - theta) < eps+1)
			withinAngle = true;
		else
            withinAngle = false;
        if (Math.abs(mLimelight.getDesiredTargetArea()-mLimelight.getTargetArea()) < 0.5)//% area
			withinRange = true;
		else
			withinRange = false;
		if (Math.abs(getHeadingDegree() - theta) < eps)
            x = 0;
            
        setOpenLoop(new DriveSignal(yOutput + x, yOutput - x));
	}

    public boolean isWithinRange() {
        return withinRange;
    }
    public boolean IsWithinAngle() {
        return withinAngle;
    }
    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            mShifter.set(wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    public PIDWrapper getDriveStraightPID() {
        if (mLimelight.getLimelightMode())
            return straightPID;
        else
            return straightPID2;
	}

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized double getHeadingDegree() {
        return mPigeon.getFusedHeading();
    }

    public synchronized double getRoll() {
        return ypr[2];
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        //SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        //SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        //SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        //SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        /*SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
        SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
        SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());
        mPigeon.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Gyro Roll", ypr[2]); // roll is the ting i want*/
        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", mPigeon.getFusedHeading());//getHeading().getDegrees()
        }
        //if (mCSVWriter != null) {
           // mCSVWriter.write();
        //}
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        mPigeon.setFusedHeading(0);
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance()+getRightEncoderDistance())/2;
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                //setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.config_kP(kVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
        
        mLeftMaster.config_kP(kPositionControlSlot, 2.5, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kPositionControlSlot, 0.0, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kPositionControlSlot, 6.0, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kPositionControlSlot, 0.2, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kPositionControlSlot, 2.5, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kPositionControlSlot, 0.0, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kPositionControlSlot, 6.0, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kPositionControlSlot, 0.2, Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        }

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0) {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        }

        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        POSITION
    }

    public enum ShifterState {
        FORCE_LOW_GEAR,
        FORCE_HIGH_GEAR
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
}

