// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.vendors.firstparties.Clock;
import frc.robot.hardware.vendors.firstparties.Settings;
import frc.robot.subsystems.WC;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Optional;

import com.flybotix.hfr.codex.CodexMetadata;
import com.flybotix.hfr.codex.ICodexTimeProvider;
import com.flybotix.hfr.util.log.ELevel;
import com.flybotix.hfr.util.log.ILog;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 * If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */

@SuppressWarnings("PMD.CommentSize")
public class Robot extends TimedRobot {
  private Command mAutonomousCommand;
  private final SysIdRoutineBot mRobot = new SysIdRoutineBot();
  private final PowerDistribution mPDP = new PowerDistribution();
  static final edu.wpi.first.wpilibj.I2C.Port kPort = edu.wpi.first.wpilibj.I2C.Port.kOnboard;
  private static final int kDeviceAddress = 4;

  private final I2C mArduino = new I2C(kPort, kDeviceAddress);

  private void writeString(String input) {
    // Creates a char array from the input string
    char[] chars = input.toCharArray();

    // Creates a byte array from the char array
    byte[] data = new byte[chars.length];

    // Adds each character
    for (int i = 0; i < chars.length; i++) {
      data[i] = (byte) chars[i];
    }

    // Writes bytes over I2C
    mArduino.transaction(data, data.length, new byte[] {}, 0);
  }
  

  private ILog mLogger = com.flybotix.hfr.util.log.Logger.createLog(this.getClass());
  public static final Clock CLOCK = (RobotBase.isReal() ? new Clock() : new Clock().simulated());
  public static final Field2d FIELD = new Field2d();
  public static final boolean IS_SIMULATED = RobotBase.isSimulation();
  public static String CLIMB_MODE = "";
  private final Settings mSettings = new Settings();
  private Timer initTimer = new Timer();
  private WC mWestCoastDrive;
  private static final java.util.logging.Logger LOGGER = java.util.logging.Logger
      .getLogger(Robot.class.getName());
  public static String trajectoryJSON = "Paths/output/PathWeaver_Straight.wpilib.json";
  public static Trajectory trajectory = new Trajectory();

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
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    SmartDashboard.putData("PDP", mPDP);
  
    mRobot.configureBindings();

    CLOCK.update();
    mLogger.warn("===> ROBOT INIT Starting");
    mWestCoastDrive = WC.getInstance();

    com.flybotix.hfr.util.log.Logger.setLevel(ELevel.WARN);
    mLogger.info("Starting Robot Initialization...");
    ICodexTimeProvider provider = new ICodexTimeProvider() {
      @Override
      public double getTimestamp() {
        return CLOCK.now();
      }
    };
    CodexMetadata.overrideTimeProvider(provider);

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
    sigma();
  }

  public void sigma() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  /**
   * These things rely on match metadata, so we need to wait for the DS to connect
   */
  private void initAfterConnection() {
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
    
    // Get the current going through channel 7, in Amperes.
    // The PDP returns the current in increments of 0.125A.
    // At low currents the current readings tend to be less accurate.
    double lCurrent7 = mPDP.getCurrent(7);
    SmartDashboard.putNumber("Current Channel 7", lCurrent7);

    // Get the voltage going into the PDP, in Volts.
    // The PDP returns the voltage in increments of 0.05 Volts.
    double voltage = mPDP.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);

    // Retrieves the temperature of the PDP, in degrees Celsius.
    double temperatureCelsius = mPDP.getTemperature();
    SmartDashboard.putNumber("Temperature", temperatureCelsius);

    // Get the total current of all channels.
    double totalCurrent = mPDP.getTotalCurrent();
    SmartDashboard.putNumber("Total Current", totalCurrent);

    // Get the total power of all channels.
    // Power is the bus voltage multiplied by the current with the units Watts.
    double totalPower = mPDP.getTotalPower();
    SmartDashboard.putNumber("Total Power", totalPower);

    // Get the total energy of all channels.
    // Energy is the power summed over time with units Joules.
    double totalEnergy = mPDP.getTotalEnergy();
    SmartDashboard.putNumber("Total Energy", totalEnergy);

    // Creates a string to hold current robot state information, including alliance, enabled state, operation mode, and match time. The message is sent in format "AEM###" where A is the alliance color, (R)ed or (B)lue, E is the enabled state, (E)nabled or (D)isabled, M is the operation mode, (A)utonomous or (T)eleop, and ### is the zero-padded time remaining in the match.
    //
    // For example, "RET043" would indicate that the robot is on the red alliance, enabled in teleop mode, with 43 seconds left in the match.
    StringBuilder stateMessage = new StringBuilder(6);

    String allianceString = "U";
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      allianceString = alliance.get() == DriverStation.Alliance.Red ? "R" : "B";
    }

    stateMessage
        .append(allianceString)
        .append(DriverStation.isEnabled() ? "E" : "D")
        .append(DriverStation.isAutonomous() ? "A" : "T")
        .append(String.format("%03d", (int) DriverStation.getMatchTime()));

    writeString(stateMessage.toString());
  }

  /** Close all resources. */
  @Override
  public void close() {
    mArduino.close();
    super.close();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    mLogger.info("Disabled Initialization");
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    mAutonomousCommand = mRobot.getAutonomousCommand();
    if(mAutonomousCommand != null) {
      mAutonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running.
    // If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
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
