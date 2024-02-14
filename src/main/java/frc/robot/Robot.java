// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.common.config.AbstractSystemSettingsUtils;
import frc.common.network.Network;
import frc.common.types.EMatchMode;
import frc.common.types.MatchMetadata;
import frc.logging.CSVLogger;
import frc.logging.Log;
import frc.robot.controller.BaseAutonController;
import frc.robot.hardware.vendors.firstparties.Clock;
import frc.robot.hardware.vendors.firstparties.Data;
import frc.robot.hardware.vendors.firstparties.Settings;
import frc.robot.network.EForwardableConnections;
import frc.robot.subsystems.ModuleList;
import frc.robot.subsystems.WestCoastDrive;
import lombok.Getter;

import java.util.Arrays;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;
import com.flybotix.hfr.codex.RobotCodex;
import com.flybotix.hfr.util.log.ELevel;
import com.flybotix.hfr.util.log.ILog;

/**
 * Main class of the project. Responsible for initializing and controlling
 * the robot in different modes such as autonomous and teleoperated.
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 * 
 * If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */

@SuppressWarnings("PMD.CommentSize")
public class Robot extends TimedRobot {
  private ILog mLogger = com.flybotix.hfr.util.log.Logger.createLog(this.getClass());
  public static final Data DATA = new Data();
  public static final Clock CLOCK = (RobotBase.isReal() ? new Clock() : new Clock().simulated());
  public static final Field2d FIELD = new Field2d();
  public static final boolean IS_SIMULATED = RobotBase.isSimulation();
  private static EMatchMode MODE = EMatchMode.DISABLED;
  public static String CLIMB_MODE = "";
  private ModuleList mRunningModules = new ModuleList();
  private final Settings mSettings = new Settings();
  private CSVLogger mCSVLogger;
  private Timer initTimer = new Timer();

  private WestCoastDrive mWestCoastDrive;
  // private AutonSelection mAutonSelection;
  private ClimbModeSelection mClimberSelector;
  private BaseAutonController mBaseAutonController;
  private InputMap mOI;
  private MatchMetadata mMatchMeta = null;
  private @Getter Command autonomousCommand;
  private static final @Getter java.util.logging.Logger LOGGER = java.util.logging.Logger
      .getLogger(Robot.class.getName());

  /**
   * Default constructor for the Robot class. This constructor is automatically
   * invoked when an instance of the Robot class is created.
   * Initializes the Robot instance by calling the no-argument constructor of the
   * superclass (TimedRobot).
   */
  public Robot() {
    super();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.
    // This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    RobotContainer robotContainer = new RobotContainer();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setFPS(30);
    CLOCK.update();
    Arrays.stream(EForwardableConnections.values()).forEach(EForwardableConnections::portForwarding);
    // mAutonSelection = new AutonSelection();
    mBaseAutonController = new BaseAutonController();
    // mShootMoveController = new ShootMoveController();
    // mThreeBallAuton = new ThreeBallTrajectoryController();
    // mFourBallAuton = new FourBallTrajectoryAuton();
    MODE = EMatchMode.INITIALIZING;
    mLogger.warn("===> ROBOT INIT Starting");
    mOI = new InputMap();
    // mLEDControl = new LEDModule();
    // mAddressableLEDs = new AddressableLEDs();
    mWestCoastDrive = WestCoastDrive.getInstance();
    // mLimelight = new Limelight();
    if (IS_SIMULATED) {
      // mSimulation = new SimulationModule();
    }

    AbstractSystemSettingsUtils.loadPracticeSettings(mSettings);
    com.flybotix.hfr.util.log.Logger.setLevel(ELevel.WARN);
    mLogger.info("Starting Robot Initialization...");
    ICodexTimeProvider provider = new ICodexTimeProvider() {
      @Override
      public double getTimestamp() {
        return CLOCK.now();
      }
    };
    CodexMetadata.overrideTimeProvider(provider);

    mRunningModules.clearModules();

    LiveWindow.disableAllTelemetry();

    /*
     * Some things need to wait until after the robot connects to the DS. So keep
     * this thread here.
     */
    new Thread(new DSConnectInitThread()).start();

    initTimer.stop();
    mLogger.warn("Robot initialization finished. Took: ", initTimer.get(), " seconds");

    if (!Settings.kIsLogging) {
      mLogger.warn("------------Not Logging to CSV------------");
    }

  }

  /**
   * These things rely on match metadata, so we need to wait for the DS to connect
   */
  private void initAfterConnection() {
    initMatchMetadata();
    mCSVLogger = new CSVLogger(Settings.kIsLogging, mMatchMeta);
  }

  /**
   * This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.
    // This is responsible for polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(FIELD);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    MODE = EMatchMode.DISABLED;
    mLogger.info("Disabled Initialization");
    mRunningModules.modeInit(EMatchMode.DISABLED);

    mRunningModules.shutdown();

    // if (mActiveController != null) {
    //   mActiveController.setEnabled(false);
    // }
  }

  @Override
  public void disabledPeriodic() {
    mRunningModules.safeReadInputs();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    RobotContainer robotContainer = new RobotContainer();
    MODE = EMatchMode.AUTONOMOUS;
    //Robot.DATA.registerAllWithShuffleboard();
    mRunningModules.clearModules();
    mRunningModules.addModule(mWestCoastDrive);
    // mRunningModules.modeInit(EMatchMode.AUTONOMOUS);
    // mWestCoastDrive.readInputs();
    // if (mAutonSelection.getSelectedAutonController() != null) {
    //    mAutonSelection.getSelectedAutonController().schedule();
    // }

    autonomousCommand = robotContainer.getAutonomousCommand();

    // Schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    commonPeriodic();
  }

  @Override
  public void teleopInit() {
    if ( Settings.kIsLogging ){
      mCSVLogger.start();
    }
    mRunningModules.clearModules();
    // mRunningModules.addModule(mOI);
    mRunningModules.addModule(mWestCoastDrive);
    MODE = EMatchMode.TELEOPERATED;
    // mActiveController = mTeleopController;
    // mActiveController.setEnabled(true);
    mRunningModules.modeInit(EMatchMode.TELEOPERATED);

    // This makes sure that the autonomous stops running when teleop starts running.
    // If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    commonPeriodic();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    Robot.DATA.registerAllWithShuffleboard();
    teleopInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    commonPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // ...
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // ...
  }

  void commonPeriodic() {
    double start = Timer.getFPGATimestamp();
    CLOCK.update();
    if (Settings.kIsLogging && MODE != EMatchMode.DISABLED) {
      for (RobotCodex c : DATA.mLoggedCodexes) {
        mCSVLogger.addToQueue(new Log(c.toFormattedCSV(), c.meta().gid()));
      }
    }
    for (RobotCodex c : DATA.mAllCodexes) {
      c.reset();
    }

    mRunningModules.safeReadInputs();

    mRunningModules.safeSetOutputs();
    SmartDashboard.putNumber("common_periodic_dt", Timer.getFPGATimestamp() - start);
    SmartDashboard.putNumber("Clock Time", CLOCK.now());
  }

  private void initMatchMetadata() {
    if (mMatchMeta == null) {
      mMatchMeta = new MatchMetadata();
      int gid = mMatchMeta.hash;
      for (RobotCodex c : DATA.mAllCodexes) {
        c.meta().setGlobalId(gid);
      }
    }
  }

  public static EMatchMode mode() {
    return MODE;
  }

  public String toString() {

    String mRobotMode = "Unknown";
    String mRobotEnabledDisabled = "Unknown";
    double mNow = Timer.getFPGATimestamp();

    if (this.isAutonomous()) {
      mRobotMode = "Autonomous";
    }
    if (this.isTest()) {
      mRobotEnabledDisabled = "Test";
    }
    if (this.isEnabled()) {
      mRobotEnabledDisabled = "Enabled";
    }
    if (this.isDisabled()) {
      mRobotEnabledDisabled = "Disabled";
    }

    return String.format("State: %s\tMode: %s\tTime: %s", mRobotEnabledDisabled, mRobotMode, mNow);

  }

  private class DSConnectInitThread implements Runnable {

    @Override
    public void run() {
      while (!DriverStation.isDSAttached()) {
        try {
          mLogger.error("Waiting on Robot <--> DS Connection...");
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
      initAfterConnection();
    }
  }
}
