package frc.robot.subsystems;

public class Lifter extends Subsystem {
        private static Lifter mInstance = null;
        private boolean mDeployed;

        private Lifter() {

        // Start the forklift in the retracted position.  Set true to force a state change.
        mDeployed = true;
        retract();
        }

        public synchronized static Lifter getInstance() {
        if (mInstance == null) {
                mInstance = new Lifter();
        }
        return mInstance;
        }

        public synchronized void deploy() {
        // Try to avoid hitting CAN/JNI wrapper.
        if (!mDeployed) {
                mDeployed = true;
        }
        }

        public synchronized void retract() {
        // Try to avoid hitting CAN/JNI wrapper.
        if (mDeployed) {
                mDeployed = false;
        }
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
        }

        @Override
        public void zeroSensors() {
        }
}