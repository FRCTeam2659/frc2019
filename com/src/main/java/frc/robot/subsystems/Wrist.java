package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Wrist extends Subsystem {
        private static final int kMagicMotionSlot = 0;
        private static final int kPositionControlSlot = 1;

        private int kHomeAngle = 35+10; //35
        private boolean mHasBeenZeroed = false;
    
        private static Wrist mInstance;
        private final TalonSRX mMaster;
        private PeriodicIO mPeriodicIO = new PeriodicIO();
        private double mZeroPosition = Double.NaN;
        private SystemState mSystemState = SystemState.HOMING;
        private SystemState mDesiredState = SystemState.MOTION_PROFILING;
    
        private Wrist() {
            mMaster = TalonSRXFactory.createDefaultTalon(Constants.kWristMasterId);
    
            //configure talon
            mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
            mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
    
            //configure magic motion
            mMaster.config_kP(kMagicMotionSlot, Constants.kWristKp, Constants.kLongCANTimeoutMs);
            mMaster.config_kI(kMagicMotionSlot, Constants.kWristKi, Constants.kLongCANTimeoutMs);
            mMaster.config_kD(kMagicMotionSlot, Constants.kWristKd, Constants.kLongCANTimeoutMs);
            mMaster.config_kF(kMagicMotionSlot, Constants.kWristKf, Constants.kLongCANTimeoutMs);
            mMaster.configAllowableClosedloopError(kMagicMotionSlot, Constants.kWristDeadband, Constants.kLongCANTimeoutMs);
            mMaster.configMotionAcceleration(Constants.kWristAcceleration, Constants.kLongCANTimeoutMs);
            mMaster.configMotionCruiseVelocity(Constants.kWristCruiseVelocity, Constants.kLongCANTimeoutMs);

            // Configure position PID
            mMaster.config_kP(kPositionControlSlot, Constants.kWristJogKp, Constants.kLongCANTimeoutMs);
            mMaster.config_kI(kPositionControlSlot, Constants.kWristKi, Constants.kLongCANTimeoutMs);
            mMaster.config_kD(kPositionControlSlot, Constants.kWristJogKd, Constants.kLongCANTimeoutMs);
            mMaster.configAllowableClosedloopError(kPositionControlSlot, Constants.kWristDeadband, Constants.kLongCANTimeoutMs);
            mMaster.configContinuousCurrentLimit(30, Constants.kLongCANTimeoutMs);
            mMaster.configPeakCurrentLimit(40, Constants.kLongCANTimeoutMs);
            mMaster.configPeakCurrentDuration(200, Constants.kLongCANTimeoutMs);
            mMaster.configClosedloopRamp(Constants.kWristRampRate, Constants.kLongCANTimeoutMs);
            mMaster.enableCurrentLimit(true);
    
            mMaster.selectProfileSlot(0, 0);
    
            mMaster.setInverted(true);
            mMaster.setSensorPhase(false);
            mMaster.setNeutralMode(NeutralMode.Brake);
    
            mMaster.enableVoltageCompensation(true);
            mMaster.set(ControlMode.PercentOutput, 0);
    
            mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
            mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
        }
    
        public synchronized static Wrist getInstance() {
            if (mInstance == null) {
                mInstance = new Wrist();
            }
            return mInstance;
        }
    
        @Override
        public synchronized void outputTelemetry() {
            SmartDashboard.putNumber("Wrist Angle", getAngle());
            SmartDashboard.putNumber("Wrist Position", getPosition());
            SmartDashboard.putNumber("Wrist Ticks", mPeriodicIO.position_ticks);
            /*SmartDashboard.putNumber("Wrist periodic demand", mPeriodicIO.demand);
    
            SmartDashboard.putNumber("Wrist RPM", getRPM());
            SmartDashboard.putNumber("Wrist Power %", mPeriodicIO.output_percent);
            SmartDashboard.putBoolean("Wrist Has Sent Trajectory", hasFinishedTrajectory());
            SmartDashboard.putNumber("Wrist feedforward", mPeriodicIO.feedforward);
    
            if (mCSVWriter != null) {
                mCSVWriter.write();
            }*/
        }
    
        @Override
        public void stop() {
            setOpenLoop(0.0);
        }
    
        @Override
        public synchronized void zeroSensors() {
            mMaster.setSelectedSensorPosition(0, 0, 0);
            mHasBeenZeroed = true;
        }
    
        public synchronized boolean hasBeenZeroed() {
            return mHasBeenZeroed;
        }
    
        @Override
        public void registerEnabledLoops(ILooper enabledLooper) {
            enabledLooper.register(new Loop() {
    
                @Override
                public void onStart(double timestamp) {
                    // startLogging();
                }
    
                @Override
                public void onLoop(double timestamp) {
                    synchronized (Wrist.this) {
                        if (!Double.isNaN(mZeroPosition) && mDesiredState != mSystemState) {
                            System.out.println(timestamp + ": Wrist changed states: " + mSystemState + " -> " +
                                    mDesiredState);
                            mSystemState = mDesiredState;
                        }
    
                        switch (mSystemState) {
                            case OPEN_LOOP:
                                // Handled in writePeriodicOutputs
                                break;
                            case MOTION_PROFILING:
                                // Handled in writePeriodicOutputs
                                break;
                            case HOMING:
                                // TODO get this working again
    //                            if (Double.isNaN(mZeroPosition)) {
    //                                mPeriodicOutputs.demand = resetIfAtLimit() ? 0.0 : kHomingOutput;
    //                            } else {
                                mSystemState = SystemState.OPEN_LOOP;
    
                                break;
                            default:
                                System.out.println("Fell through on Wrist states!");
                        }
                    }
                }
    
                @Override
                public void onStop(double timestamp) {
                }
            });
        }
    
        public synchronized void setOpenLoop(double percentage) {
            mPeriodicIO.demand = percentage;
            mDesiredState = SystemState.OPEN_LOOP;
        }
    
        public synchronized boolean resetIfAtLimit() {
            /*if (mCanifier.getLimR()) {
                zeroSensors();
                return true;
            }*/
            return false;
        }

        public synchronized void setHomeAngle(int relativeAngle) {
            kHomeAngle += relativeAngle;
        }
    
        /**
         * @param position the target position of the wrist in sensor units
         */
        public void setClosedLoop(int position) {
            mPeriodicIO.demand = (position);
            mDesiredState = SystemState.MOTION_PROFILING;
        }
    
        /**
         * @param angle the target position of the wrist in degrees.  0 is full back, 180 is facing forwards
         */
        public synchronized void setMotionProfileAngle(double angle) {
            double angleFromHome = angle - kHomeAngle;
            mPeriodicIO.demand = (degreesToSensorUnits(angleFromHome));
            if (mDesiredState != SystemState.MOTION_PROFILING) {
                mDesiredState = SystemState.MOTION_PROFILING;
                mMaster.selectProfileSlot(kMagicMotionSlot, 0);
            }
        }
    
        /**
         * @param angle the target position of the wrist in degrees.  0 is full back, 180 is facing forwards
         */
        public synchronized void setPositionPIDAngle(double angle) {
            double angleFromHome = angle - kHomeAngle;
            mPeriodicIO.demand = (degreesToSensorUnits(angleFromHome));
            if (mDesiredState != SystemState.POSITION_PID) {
                mDesiredState = SystemState.POSITION_PID;
                mMaster.selectProfileSlot(kPositionControlSlot, 0);
            }
        }
    
        public synchronized void setJogWrist(double relativeAngle) {
            setPositionPIDAngle(getAngle() + relativeAngle);
        }

        /**
         * @return current position of the wrist in sensor units
         */
        public synchronized double getPosition() { //returns angle of wrist in degrees
            return (mPeriodicIO.position_ticks) + degreesToSensorUnits(kHomeAngle);
        }
    
        /**
         * @return current angle of the wrist in degrees
         */
        public synchronized double getAngle() { //returns angle of wrist in degrees
            return kHomeAngle + sensorUnitsToDegrees((mPeriodicIO.position_ticks));
        }
    
        /**
         * @return current velocity in rpm
         */
        public double getRPM() {
            return sensorUnitsToDegrees(mPeriodicIO.velocity_ticks_per_100ms) * 600.0 / 360.0;
        }
    
        /**
         * @return current velocity in degrees per second
         */
        public double getDegreesPerSecond() {
            return sensorUnitsToDegrees(mPeriodicIO.velocity_ticks_per_100ms) * 10.0;
        }
    
        public synchronized boolean hasFinishedTrajectory() {
            if (Util.epsilonEquals(mPeriodicIO.active_trajectory_position,
                    degreesToSensorUnits(getSetpoint()), 2)) {
                return true;
            }
            return false;
        }
    
        public synchronized double getSetpoint() {
            return mDesiredState == SystemState.MOTION_PROFILING || mDesiredState == SystemState.POSITION_PID
                    ? sensorUnitsToDegrees((mPeriodicIO.demand)) : Double.NaN;
        }
    
        private double sensorUnitsToDegrees(double units) {
            return units / 4096.0 * 360.0 / 6.93333;
        }
    
        private double degreesToSensorUnits(double degrees) {
            return degrees * 4096.0 / 360.0 * 6.93333;
        }
    
        @Override
        public synchronized void readPeriodicInputs() {
            if (mMaster.getControlMode() == ControlMode.MotionMagic) {
                mPeriodicIO.active_trajectory_position = mMaster.getActiveTrajectoryPosition();
                int newVel = mMaster.getActiveTrajectoryVelocity();
                // TODO check sign of accel
                if (Util.epsilonEquals(newVel, Constants.kWristCruiseVelocity, 5) || Util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, 5)) {
                    // Wrist is ~constant velocity.
                    mPeriodicIO.active_trajectory_acceleration_rad_per_s2 = 0.0;
                } else {
                    // Wrist is accelerating.
                    mPeriodicIO.active_trajectory_acceleration_rad_per_s2 = Math.signum(newVel - mPeriodicIO
                            .active_trajectory_velocity) * Constants.kWristAcceleration * 20.0 * Math.PI /
                            4096;
                }
                mPeriodicIO.active_trajectory_velocity = newVel;
            } else {
                mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
                mPeriodicIO.active_trajectory_velocity = 0;
                mPeriodicIO.active_trajectory_acceleration_rad_per_s2 = 0.0;
            }
            //mPeriodicIO.output_voltage = mMaster.getMotorOutputVoltage();
            //mPeriodicIO.output_percent = mMaster.getMotorOutputPercent();
            mPeriodicIO.position_ticks = mMaster.getSelectedSensorPosition(0);
            mPeriodicIO.velocity_ticks_per_100ms = mMaster.getSelectedSensorVelocity(0);
    
            if (getAngle() > Constants.kWristEpsilon || sensorUnitsToDegrees(mPeriodicIO.active_trajectory_position) > Constants.kWristEpsilon) {
                double wristGravityComponent = Math.sin(Math.toRadians(getAngle() - (180 - Elbow.getInstance().getAngle()))) * Constants.kWristKfMultiplier;
                double wristAccelerationComponent = mPeriodicIO.active_trajectory_acceleration_rad_per_s2 * Constants.kWristKa;
                mPeriodicIO.feedforward = wristGravityComponent + wristAccelerationComponent;
            } else {
                if (getSetpoint() < Util.kEpsilon) {
                    mPeriodicIO.feedforward = -0.1;
                } else {
                    mPeriodicIO.feedforward = 0.0;
                }
            }
        }
    
        @Override
        public synchronized void writePeriodicOutputs() {
            if (mDesiredState == SystemState.MOTION_PROFILING) {
                mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
            } else if (mDesiredState == SystemState.POSITION_PID) {
                mMaster.set(ControlMode.Position, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
            } else {
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
            }
        }
    
        @Override
        public boolean checkSystem() {
                return true;
        }
    

    
        public enum SystemState {
            HOMING,
            MOTION_PROFILING,
            POSITION_PID,
            OPEN_LOOP,
        }
    
        public static class PeriodicIO {
            // INPUTS
            public int position_ticks;
            public int velocity_ticks_per_100ms;
            public int active_trajectory_position;
            public int active_trajectory_velocity;
            public double active_trajectory_acceleration_rad_per_s2;
            public double output_percent;
            public double output_voltage;
            public double feedforward;
    
            // OUTPUTS
            public double demand;
        }
    }