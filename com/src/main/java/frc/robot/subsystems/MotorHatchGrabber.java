package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class MotorHatchGrabber extends Subsystem {
        private static MotorHatchGrabber mInstance;
        private final TalonSRX mJointMaster;
        private final VictorSPX mIntakeMaster;
    
        private MotorHatchGrabber() {
            mJointMaster = TalonSRXFactory.createDefaultTalon(Constants.kHatchIntakeMasterId);
            mJointMaster.set(ControlMode.PercentOutput, 0);
            mJointMaster.setInverted(true);
            mJointMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
            mJointMaster.enableVoltageCompensation(true);
            mJointMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
            mJointMaster.config_kP(0, Constants.kHatchJointKp, Constants.kLongCANTimeoutMs);
            mJointMaster.config_kI(0, Constants.kHatchJointKi, Constants.kLongCANTimeoutMs);
            mJointMaster.config_kD(0, Constants.kHatchJointKd, Constants.kLongCANTimeoutMs);
            mJointMaster.config_kF(0, Constants.kHatchJointKf, Constants.kLongCANTimeoutMs);
            mJointMaster.configMotionAcceleration(Constants.kHatchJointAcceleration, Constants.kLongCANTimeoutMs);
            mJointMaster.configMotionCruiseVelocity(Constants.kHatchJointCruiseVelocity, Constants.kLongCANTimeoutMs);

            mJointMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
            mJointMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
            mJointMaster.setNeutralMode(NeutralMode.Brake);
            mIntakeMaster = new VictorSPX(Constants.kHatchIntakeMasterId);
        }
    
        public synchronized static MotorHatchGrabber getInstance() {
            if (mInstance == null) {
                mInstance = new MotorHatchGrabber();
            }
            return mInstance;
        }
    
        @Override
        public void outputTelemetry() {
            
        }
    
        @Override
        public void stop() {
            setPower(0);
        }
    
        @Override
        public void zeroSensors() {
        }
    
        @Override
        public void registerEnabledLoops(ILooper enabledLooper) {
            Loop loop = new Loop() {
    
                @Override
                public void onStart(double timestamp) {
                }
    
                @Override
                public void onLoop(double timestamp) {  
                    //if (isHatchLoaded())
                        //setJoint(90);
                }
    
                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            };
            enabledLooper.register(loop);
        }
    
        public synchronized void setPower(double power) {
            mIntakeMaster.set(ControlMode.PercentOutput, power);
        }
    
        public synchronized void autoOuttake() {
            setPower(-1.0);
            setJoint(10);
        }

        public synchronized void autoIntake() {
            setPower(1.0);
            setJoint(0);
        }

        public synchronized void setJoint(int Angle) {
            mJointMaster.set(ControlMode.MotionMagic, Angle);
        }

        public boolean isHatchLoaded() {
            return false;
        }
    
        @Override
        public void readPeriodicInputs() {
        }
    
        @Override
        public void writePeriodicOutputs() {
        }
    
        @Override
        public boolean checkSystem() {
            return true;
        }
}
    