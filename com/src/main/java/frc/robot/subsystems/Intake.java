package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Intake extends Subsystem {
        private static Intake mInstance;
        private final TalonSRX mMaster;
        private final Solenoid mSolenoid;
        private Ultrasonic mUltrasonic;
        private boolean isHatchClamped = false;
    
        private Intake() {
            mMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.setInverted(true);
            mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
            mMaster.enableVoltageCompensation(true);

            mSolenoid = new Solenoid(Constants.kIntakeSolenoidId);
            mUltrasonic = new Ultrasonic(3, 0);
        }
    
        public synchronized static Intake getInstance() {
            if (mInstance == null) {
                mInstance = new Intake();
            }
            return mInstance;
        }
    
        @Override
        public void outputTelemetry() {
            SmartDashboard.putNumber("ultrasonic distance", mUltrasonic.getRangeInches());
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
                }
    
                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            };
            enabledLooper.register(loop);
        }
    
        public synchronized void setPower(double power) {
            mMaster.set(ControlMode.PercentOutput, power);
        }
    
        public synchronized void strongShoot() {
            setPower(-1.0);
        }

        public synchronized void weakShoot() {
            setPower(-0.6);
        }

        public synchronized void intake() {
            setPower(1.0);
        }
    
        public void triggerHatch() {
            mSolenoid.set(isHatchClamped);
            isHatchClamped = !isHatchClamped;
        }
        public void clampHatch() {
            isHatchClamped = true;
            mSolenoid.set(false);
        }

        public void releaseHatch() {
            isHatchClamped = false;
            mSolenoid.set(true);
        }

        public boolean isHatchClamped() {
            return isHatchClamped;
        }

        public boolean isCargoLoaded() {
            if (mUltrasonic.getRangeInches() < 6)
                return true;
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
    