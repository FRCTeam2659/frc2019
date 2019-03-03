package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Constants;

public class Elevator extends Subsystem {
    public static final double kHomePositionInches = 18.5; //important constant to tune
    private static final int kHighGearSlot = 0;
    private static final int kPositionControlSlot = 1;
    private static final double kEncoderTicksPerInch = -768.0;
    private static Elevator mInstance = null;
    private final TalonSRX mMaster;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ElevatorControlState mElevatorControlState = ElevatorControlState.OPEN_LOOP;

    private boolean mHasBeenZeroed = false;

    public Elevator() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId);
        mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);

        //configure magic motion
        mMaster.config_kP(kHighGearSlot, Constants.kElevatorHighGearKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(kHighGearSlot, Constants.kElevatorHighGearKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(kHighGearSlot, Constants.kElevatorHighGearKd + Constants.kElevatorHighGearKd / 100.0, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(kHighGearSlot, Constants.kElevatorHighGearKf, Constants.kLongCANTimeoutMs);
        mMaster.configAllowableClosedloopError(kHighGearSlot, Constants.kElevatorHighGearDeadband, Constants.kLongCANTimeoutMs);
        mMaster.configMotionAcceleration(Constants.kElevatorHighGearAcceleration, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(Constants.kElevatorHighGearCruiseVelocity, Constants.kLongCANTimeoutMs);

        //configure position PID
        mMaster.config_kP(kPositionControlSlot, Constants.kElevatorJogKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(kPositionControlSlot, Constants.kElevatorHighGearKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(kPositionControlSlot, Constants.kElevatorJogKd, Constants.kLongCANTimeoutMs);
        mMaster.configAllowableClosedloopError(kPositionControlSlot, Constants.kElevatorHighGearDeadband, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(Constants.kElevatorRampRate, Constants.kLongCANTimeoutMs);
        mMaster.configOpenloopRamp(Constants.kElevatorRampRate, Constants.kLongCANTimeoutMs);
        mMaster.configContinuousCurrentLimit(20, Constants.kLongCANTimeoutMs);
        mMaster.configPeakCurrentLimit(35, Constants.kLongCANTimeoutMs);
        mMaster.configPeakCurrentDuration(200, Constants.kLongCANTimeoutMs);
        mMaster.enableCurrentLimit(true);

        mMaster.selectProfileSlot(0, 0); //motion magic slot

        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(false);

        mMaster.enableVoltageCompensation(true);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);

        mMaster.setInverted(false);
        mMaster.setSensorPhase(true);

        // Start with zero power.
        mMaster.set(ControlMode.PercentOutput, 0);
        setNeutralMode(NeutralMode.Brake);
    }

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public synchronized void setOpenLoop(double percentage) {
        mElevatorControlState = ElevatorControlState.OPEN_LOOP;
        mPeriodicIO.demand = percentage;
    }

    public synchronized void setMotionMagicPosition(double positionInchesOffGround) {
        double positionInchesFromHome = positionInchesOffGround - kHomePositionInches;
        double encoderPosition = positionInchesFromHome * kEncoderTicksPerInch;
        setClosedLoopRawPosition(encoderPosition);
    }

    public synchronized void setPositionPID(double positionInchesOffGround) {
        double positionInchesFromHome = positionInchesOffGround - kHomePositionInches;
        double encoderPosition = positionInchesFromHome * kEncoderTicksPerInch;
        if (mElevatorControlState != ElevatorControlState.POSITION_PID) {
            mElevatorControlState = ElevatorControlState.POSITION_PID;
            mMaster.selectProfileSlot(kPositionControlSlot, 0);
        }
        mPeriodicIO.demand = encoderPosition;
    }

    public synchronized void setJogElevator(double relativePosition) {
        setPositionPID(getInchesOffGround() + relativePosition);
    }

    private synchronized void setClosedLoopRawPosition(double encoderPosition) {
        if (mElevatorControlState != ElevatorControlState.MOTION_MAGIC) {
            mElevatorControlState = ElevatorControlState.MOTION_MAGIC;
            mMaster.selectProfileSlot(kHighGearSlot, 0);
        }
        mPeriodicIO.demand = encoderPosition;
    }

    public synchronized double getRPM() {
        // We are using a CTRE mag encoder which is 4096 native units per revolution.
        return mPeriodicIO.velocity_ticks_per_100ms * 10.0 / 4096.0 * 60.0;
    }

    public synchronized double getInchesOffGround() {
        return (mPeriodicIO.position_ticks / kEncoderTicksPerInch) + kHomePositionInches;
    }

    public synchronized double getSetpoint() {
        return mElevatorControlState == ElevatorControlState.MOTION_MAGIC ?
                mPeriodicIO.demand / kEncoderTicksPerInch + kHomePositionInches : Double.NaN;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Elevator Output %", mPeriodicIO.output_percent);
        SmartDashboard.putNumber("Elevator RPM", getRPM());
        SmartDashboard.putNumber("Elevator Current", mMaster.getOutputCurrent());
        // SmartDashboard.putNumber("Elevator Error", mMaster.getClosedLoopError(0) / kEncoderTicksPerInch);
        SmartDashboard.putNumber("Elevator Height", getInchesOffGround());
        SmartDashboard.putBoolean("Elevator Limit", mPeriodicIO.limit_switch);
        SmartDashboard.putNumber("Elevator Sensor Height", mPeriodicIO.position_ticks);

        SmartDashboard.putNumber("Elevator Last Expected Trajectory", mPeriodicIO.demand);
        SmartDashboard.putNumber("Elevator Traj Vel", mMaster.getSelectedSensorVelocity(0));
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
        mHasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    public synchronized void resetIfAtLimit() {
        if (mPeriodicIO.limit_switch) {
            zeroSensors();
        }
    }

    private void setNeutralMode(NeutralMode neutralMode) {
        mMaster.setNeutralMode(neutralMode);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        final double t = Timer.getFPGATimestamp();
        mPeriodicIO.position_ticks = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.velocity_ticks_per_100ms = mMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.output_percent = mMaster.getMotorOutputPercent();
        mPeriodicIO.limit_switch = mMaster.getSensorCollection().isFwdLimitSwitchClosed();
        mPeriodicIO.t = t;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mElevatorControlState == ElevatorControlState.MOTION_MAGIC) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand);
        } else if (mElevatorControlState == ElevatorControlState.POSITION_PID) {
            mMaster.set(ControlMode.Position, mPeriodicIO.demand);
        } else {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    private enum ElevatorControlState {
        OPEN_LOOP,
        MOTION_MAGIC,
        POSITION_PID
    }

    public static class PeriodicIO {
        // INPUTS
        public int position_ticks;
        public int velocity_ticks_per_100ms;
        public double output_percent;
        public boolean limit_switch;
        public double feedforward;
        public double t;

        // OUTPUTS
        public double demand;
    }
}