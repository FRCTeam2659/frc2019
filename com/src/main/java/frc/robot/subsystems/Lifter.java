package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;

public class Lifter extends Subsystem {
        private static Lifter mInstance = null;
        private final PWMSpeedController mMaster, mSlave;
        private Lifter() {
                mMaster = new VictorSP(Constants.kLifterMasterId);
                mMaster.setInverted(true);
                mSlave = new VictorSP(Constants.kLifterSlaveId);
                mSlave.setInverted(true);
        }

        public synchronized static Lifter getInstance() {
                if (mInstance == null) {
                        mInstance = new Lifter();
                }
                return mInstance;
        }

        public synchronized void setOpenLoop(double power) {
                mMaster.set(power);
                mSlave.set(power);
                double elevatorHeight = Drive.getInstance().getRoll() + 42 - Elevator.getInstance().getInchesOffGround();
        }

        public synchronized void liftUp() {
                mMaster.set(0.75);
                mSlave.set(0.75);
        }

        public synchronized void goDown() {
                mMaster.set(-0.6);
                mSlave.set(-0.6);
        }

        @Override
        public boolean checkSystem() {
                return true;
        }

        @Override
        public void outputTelemetry() {
        }

        @Override
        public void stop() {
                mMaster.set(0);
                mSlave.set(0);
        }

        @Override
        public void zeroSensors() {
        }
}