package frc.robot;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.lib.util.TortoDriveHelper;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.Auteleop;
import frc.robot.auto.modes.CrossAutoLineMode;
import frc.robot.auto.modes.DoNothingMode;
import frc.robot.auto.modes.MidStartToFrontShip;
import frc.robot.auto.modes.Middle2HatchRocket;
import frc.robot.auto.modes.SideStartToFrontShip;
import frc.robot.auto.modes.TestMode;
import frc.robot.auto.modes.Yolo;
import frc.robot.loops.Looper;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Superstructure.SuperstructureStates;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private TortoDriveHelper mTortoDriveHelper = new TortoDriveHelper();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Limelight.getInstance(),
                    Drive.getInstance(),
                    Elevator.getInstance(),
                    Elbow.getInstance(),
                    Wrist.getInstance(),
                    Intake.getInstance(),
                    Superstructure.getInstance(),
                    Lifter.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();
    private Vision mVision = Vision.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private Elbow mElbow = Elbow.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Limelight mLimelight = Limelight.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Lifter mLifter = Lifter.getInstance();
    private Joystick mDriveStick = new Joystick(0);
    private Joystick mOperatorStick = new Joystick(1);
    private Compressor mCompressor = new Compressor();

    private AutoModeExecutor mAutoModeExecutor;
    private SendableChooser mStartPositionChooser, mAutoModeChooser;
    private boolean startOnLeft;
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private boolean mIfJog = false;
    private boolean engageStop = false;
    //private boolean level3Climb = true;
    private double climbHeight = 22;
    private double liftingPower = 0.75;
    private double liftingSecondStagePower = 0.3;
    private double elevatorPower = 0.75;
    private boolean retract = false;
    private boolean autoRetract = false;
    private boolean disableAuto = false;

    private double m_LimelightSteerCommand = 0.0;  
    private double m_LimelightDriveCommand = 0.0;
    private boolean m_LimelightHasValidTarget;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    private enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private enum AutoMode {
        DRIVE_STRAIGHT,
        YOLO,
        FRONTLASH,
        MIDLASH,
        TEST
    }

    private enum DriveControlState {
        OPEN_LOOP,
        AUTELEOP
    }

    @Override
    public void robotInit() {
        try {
            mStartPositionChooser = new SendableChooser<>();
            mStartPositionChooser.addDefault("Right", StartingPosition.RIGHT);
            mStartPositionChooser.addObject("Center", StartingPosition.CENTER);
            mStartPositionChooser.addObject("Left", StartingPosition.LEFT);
            SmartDashboard.putData("Starting Position", mStartPositionChooser);
            mAutoModeChooser = new SendableChooser<>();
            mAutoModeChooser.addDefault("2 hatch rocket", AutoMode.YOLO); // need to change this
            mAutoModeChooser.addObject("level 1 start to front cargo ship", AutoMode.MIDLASH);
            mAutoModeChooser.addObject("level 2 start to front cargo ship", AutoMode.FRONTLASH);
            mAutoModeChooser.addObject("do nothing", AutoMode.DRIVE_STRAIGHT);
            SmartDashboard.putData("Auto Mode", mAutoModeChooser);
            mAutoModeExecutor = new AutoModeExecutor();

            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setBrakeMode(false);
            mDrive.zeroSensors();
            mTrajectoryGenerator.generateTrajectories();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        //SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            //Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
            //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            mLimelight.useSecondaryCameraAsVision();
            mDisabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        //SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mCompressor.stop();
            
            //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
            Drive.getInstance().setBrakeMode(true);
            Drive.getInstance().zeroSensors();
            if (mStartPositionChooser.getSelected() == StartingPosition.RIGHT)
                startOnLeft = false;
            else if (mStartPositionChooser.getSelected() == StartingPosition.LEFT)
                startOnLeft = true;
            else if (mStartPositionChooser.getSelected() == StartingPosition.CENTER)
                mAutoModeExecutor.setAutoMode(new CrossAutoLineMode());

            if (mAutoModeChooser.getSelected() == AutoMode.FRONTLASH)
                mAutoModeExecutor.setAutoMode(new SideStartToFrontShip(startOnLeft));
            else if (mAutoModeChooser.getSelected() == AutoMode.YOLO)
                mAutoModeExecutor.setAutoMode(new Middle2HatchRocket(startOnLeft));
            else if (mAutoModeChooser.getSelected() == AutoMode.MIDLASH)
                mAutoModeExecutor.setAutoMode(new MidStartToFrontShip(startOnLeft));
            else
                mAutoModeExecutor.setAutoMode(new DoNothingMode());

            mAutoModeExecutor.start();
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
       //SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mCompressor.start();
            //RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();

            mDrive.setBrakeMode(false);
            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        //SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        //SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        if (mDriveStick.getRawButton(1)) {
            mAutoModeExecutor.stop();
            disableAuto = true;
        }
        if (disableAuto) {
            double throttle = mDriveStick.getRawAxis(1);
            double turn = mDriveStick.getRawAxis(2);
            if (mDriveStick.getRawButton(6)) {
                if (Limelight.getInstance().getTargetExists())
                    mDrive.driveLimeLightXY();
                if (mSuperstructure.getState() != SuperstructureStates.MIDDLE_POSITION && !mSuperstructure.getIsCargo()) {
                    if (mDrive.isWithinRange() || autoRetract) {
                        mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
                        autoRetract = true;
                    } else
                        mSuperstructure.setDesiredState(SuperstructureStates.VISION);
                    retract = true;
                }  
            } else {
                if (retract && !mSuperstructure.getIsCargo()) {
                    mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
                    retract = false;
                    autoRetract = false;
                }
                mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(-throttle, turn, true, mDrive.isHighGear()));
            }
            if (mOperatorStick.getRawButton(1))
                mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
            else if (mOperatorStick.getRawButton(2))
                mSuperstructure.setDesiredState(SuperstructureStates.STOWED); 
            else if (mOperatorStick.getRawButton(3))
                mSuperstructure.setDesiredState(SuperstructureStates.MIDDLE_POSITION);
            else if (mOperatorStick.getRawButton(8))
                mSuperstructure.setDesiredState(SuperstructureStates.INTAKE);
            else if (mOperatorStick.getRawButton(7))
                mSuperstructure.setDesiredOuttakeState(); 
            else
                mIntake.stop();
            if (mOperatorStick.getRawButton(6))
                mIntake.clampHatch();
            else if (mOperatorStick.getRawButton(5))
                mIntake.releaseHatch();
        }
        outputToSmartDashboard();
        try {
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
            double throttle = mDriveStick.getRawAxis(1);
            double turn = mDriveStick.getRawAxis(2);
            if (mDriveStick.getRawButton(4)) {
                //double elevatorPower = 0.5 - Math.sin(Math.toRadians(mDrive.getRoll() - 5)) * 0.5;
                //double liftingPower = 0.5 + Math.sin(Math.toRadians(mDrive.getRoll() - 5)) * 0.5;
                //idk why the elevator is reversed here
                //mLifter.setOpenLoop(liftingPower);
                if (mElevator.getInchesOffGround() < 20) {
                    mElevator.setOpenLoop(0.05);
                    mLifter.setOpenLoop(0.3);
                }
                else
                    mElevator.setOpenLoop(elevatorPower);
                //mSuperstructure.level2ClimbingPosition();
                //mLifter.liftUp();
                mLifter.setOpenLoop(liftingPower);
                // intake was taken care of below
                mElbow.setOpenLoop(0.55);
                engageStop = true;
            } else if (mDriveStick.getRawButton(2)) {
                mLifter.goDown();
                mElevator.setOpenLoop(-0.25);
                engageStop = true;
            } else if (mDriveStick.getPOV() == 0) {
                mLifter.setOpenLoop(liftingSecondStagePower); //0.35
                mSuperstructure.setClimbingRestingPosition();
                throttle -= 0.3;
                engageStop = true;
            } else if (mDriveStick.getPOV() == 180) {
                mLifter.goDown();
                mSuperstructure.setClimbingRestingPosition();
                engageStop = true;
            } else if (mDriveStick.getRawButton(10)) {
                mLifter.setOpenLoop(0.3);
                engageStop = true;
            } else if (engageStop) {
                mLifter.stop();
                mElevator.stop();
                mElbow.stop();
                mIntake.stop();
                engageStop = false;
            }

            /*if (mDriveStick.getRawButton(6)) {
                if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                    if (!mSuperstructure.getIsCargo())
                        mSuperstructure.setDesiredState(SuperstructureStates.STOWED);
                    if (mVision.isDataValid()) {
                        mAutoModeExecutor.setAutoMode(new Auteleop());
                        mAutoModeExecutor.start();
                        mDriveControlState = DriveControlState.AUTELEOP;
                    }
                }
            } else if (mDriveControlState == DriveControlState.OPEN_LOOP || mDriveStick.getRawButton(5)) {
                if (mDriveControlState == DriveControlState.AUTELEOP) {
                    mDriveControlState = DriveControlState.OPEN_LOOP;
                    mAutoModeExecutor.stop();
                }
                mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(-throttle, turn, true, mDrive.isHighGear()));
            }*/

            if (mDriveStick.getRawButton(6)) {
                //mVision.changeCameraMode(true);
                if (Limelight.getInstance().getTargetExists())
                    mDrive.driveLimeLightXY();
                if (mSuperstructure.getState() != SuperstructureStates.MIDDLE_POSITION && !mSuperstructure.getIsCargo()) {
                    if (mDrive.isWithinRange() || autoRetract) {
                        mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
                        autoRetract = true;
                    } else
                        mSuperstructure.setDesiredState(SuperstructureStates.VISION);
                    retract = true;
                }
                    
                //Update_Limelight_Tracking(); 
                /*if (!mSuperstructure.getIsCargo()) {
                    if (ty < 15 && m_LimelightHasValidTarget)
                        mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
                    else
                        mSuperstructure.setDesiredState(SuperstructureStates.STOWED);
                }*/
                //mDrive.setBrakeMode(true);
                //mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(-throttle, m_LimelightSteerCommand, true, mDrive.isHighGear()));
                
            } else {
                if (retract && !mSuperstructure.getIsCargo()) {
                    mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
                    retract = false;
                    autoRetract = false;
                }
                //mVision.changeCameraMode(false);
                //mDrive.setBrakeMode(false);
                mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(-throttle, turn, true, mDrive.isHighGear()));
            }
            /*if (mDriveStick.getRawButton(6) || mOperatorStick.getRawButton(5) || mOperatorStick.getRawButton(6))
                mVision.changeCameraMode(true);
            else
                mVision.changeCameraMode(false);*/
            
            //boolean quickTurn = Math.abs(mDrive.getLinearVelocity()) > 100 ? false : true;

            //mElevator.setOpenLoop(mOperatorStick.getRawAxis(1)); // Elbow: + = up, - = down 10% Elevator: + = up, - = down
            //mWrist.setOpenLoop(0); // wrist: + = up, - = down 10%
            if (mOperatorStick.getRawButton(1))
                mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
            else if (mOperatorStick.getRawButton(2))
                mSuperstructure.setDesiredState(SuperstructureStates.STOWED); 
            else if (mOperatorStick.getRawButton(3))
                mSuperstructure.setDesiredState(SuperstructureStates.MIDDLE_POSITION);
            else if (mOperatorStick.getRawButton(8))
                mSuperstructure.setDesiredState(SuperstructureStates.INTAKE);
            else if (mOperatorStick.getRawButton(7))
                mSuperstructure.setDesiredOuttakeState(); 
            else if (mDriveStick.getRawButton(4) && mElevator.getInchesOffGround() < climbHeight)
                mIntake.intake();
            else {
                if (mSuperstructure.getIsCargo())
                    mIntake.setPower(0.1);
                else
                    mIntake.stop();
            }

            if (mOperatorStick.getPOV() == 0 && mIfJog) {
                mSuperstructure.setElbowHomeAngle(3);
                mIfJog = false;
            } else if (mOperatorStick.getPOV() == 180 && mIfJog) {
                mSuperstructure.setElbowHomeAngle(-3);
                mIfJog = false;
            } else if (mOperatorStick.getPOV() == 90 && mIfJog) {
                mSuperstructure.setWristHomeAngle(3);
                mIfJog = false;
            } else if (mOperatorStick.getPOV() == 270 && mIfJog) {
                mSuperstructure.setWristHomeAngle(-3);
                mIfJog = false;
            } else if (mOperatorStick.getPOV() == -1) { // I can just do it once 
                mIfJog = true;
            }

            if (mOperatorStick.getRawButton(6))
                mIntake.clampHatch();
            else if (mOperatorStick.getRawButton(5))
                mIntake.releaseHatch();

            if (mOperatorStick.getRawButton(9)) // mode button
                mSuperstructure.setIsCargo(false); // isTriggered = true;
            else if (mOperatorStick.getRawButton(10))
                mSuperstructure.setIsCargo(true);

            if (mDriveStick.getRawButton(3)) {
                mSuperstructure.setLevel3ClimbingPosition();
                mDrive.setHighGear(false);
                climbHeight = 22;
                liftingPower = 0.8; //.81
                liftingSecondStagePower = 0.1;
                elevatorPower = 0.75;
                //level3Climb = true;
            }
            if (mDriveStick.getRawButton(1)) {
                mSuperstructure.setLevel2ClimbingPosition();
                mDrive.setHighGear(false);
                climbHeight = 28.5;
                liftingPower = 0.4;//0.35
                liftingSecondStagePower = 0.6;
                elevatorPower = 0.6;
                //level3Climb = false;
            }
            
            if (mDriveStick.getRawButton(8))
                mDrive.setHighGear(true);
            else if (mDriveStick.getRawButton(7))
                mDrive.setHighGear(false);

            outputToSmartDashboard();
    }

    public Joystick getOperatorStick() {
        return mOperatorStick;
    }

    @Override
    public void testPeriodic() {
        //SmartDashboard.putString("Match Cycle", "TEST");
    }

    private void Update_Limelight_Tracking() {
        final double STEER_K = 0.07;                    // how hard to turn toward the target
        final double DRIVE_K = 0.2;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 5.6;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }

    private void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputTelemetry();
        Limelight.getInstance().outputTelemetry();
        //Elevator.getInstance().outputTelemetry();
        Elbow.getInstance().outputTelemetry();
        Wrist.getInstance().outputTelemetry();
        //Intake.getInstance().outputTelemetry();
        //mMotorHatchGrabber.outputTelemetry();
        //mEnabledLooper.outputToSmartDashboard();
        // SmartDashboard.updateValues();
    }
}
