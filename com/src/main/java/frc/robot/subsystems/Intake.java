package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Intake extends Subsystem {
        private static Intake mInstance;
        //private final VictorSPX mMaster;
        private final TalonSRX mMaster;
        private final Solenoid mSolenoid;
        private final Solenoid mPusherSolenoid;
        private boolean isHatchClamped = false;
    
        private Intake() {
            //mMaster = new VictorSPX(Constants.kIntakeMasterId);
            mMaster = new TalonSRX(Constants.kIntakeMasterId);
            mMaster.set(ControlMode.PercentOutput, 0);
            mMaster.setInverted(true);
            mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
            mMaster.enableVoltageCompensation(true);

            mSolenoid = new Solenoid(Constants.kIntakeSolenoidId);
            mPusherSolenoid = new Solenoid(Constants.kPusherSolenoidId);
        }
    
        public synchronized static Intake getInstance() {
            if (mInstance == null) {
                mInstance = new Intake();
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
            setPower(-0.8);
        }

        public synchronized void weakShoot() {
            setPower(-0.6);
        }

        public synchronized void intake() {
            setPower(1.0);
        }
    
        public void clampHatch() {
            isHatchClamped = true;
            mSolenoid.set(false);
            restorePusher();
        }

        public void releaseHatch() {
            isHatchClamped = false;
            mSolenoid.set(true);
            autoPush();
        }

        public void autoPush() {
            actuatePusher();
            Timer.delay(0.35);
            restorePusher();
        }

        public void actuatePusher() {
            mPusherSolenoid.set(true);
        }

        public void restorePusher() {
            mPusherSolenoid.set(false);
        }

        public boolean isHatchClamped() {
            return isHatchClamped;
        }

        public boolean isCargoLoaded() {
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
    