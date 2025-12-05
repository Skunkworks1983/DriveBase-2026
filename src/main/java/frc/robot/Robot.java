// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // (robotIP):5801 will now point to a Limelight3A's (id 0) web interface stream:
    // (robotIP):5800 will now point to a Limelight3A's (id 0) video stream:
    PortForwarder.add(5800, "172.29.0.1", 5800);
    PortForwarder.add(5801, "172.29.0.1", 5801);
    PortForwarder.add(5802, "172.29.0.1", 5802);
    PortForwarder.add(5803, "172.29.0.1", 5803);
    PortForwarder.add(5804, "172.29.0.1", 5804);
    PortForwarder.add(5805, "172.29.0.1", 5805);
    PortForwarder.add(5806, "172.29.0.1", 5806);
    PortForwarder.add(5807, "172.29.0.1", 5807);
    PortForwarder.add(5808, "172.29.0.1", 5808);
    PortForwarder.add(5809, "172.29.0.1", 5809);

    // (robotIP):5811 will now point to a Limelight3A's (id 1) web interface stream:
    // (robotIP):5810 will now point to a Limelight3A's (id 1) video stream:
    PortForwarder.add(5810, "172.29.1.1", 5800);
    PortForwarder.add(5811, "172.29.1.1", 5801);
    PortForwarder.add(5812, "172.29.1.1", 5802);
    PortForwarder.add(5813, "172.29.1.1", 5803);
    PortForwarder.add(5814, "172.29.1.1", 5804);
    PortForwarder.add(5815, "172.29.1.1", 5805);
    PortForwarder.add(5816, "172.29.1.1", 5806);
    PortForwarder.add(5817, "172.29.1.1", 5807);
    PortForwarder.add(5818, "172.29.1.1", 5808);
    PortForwarder.add(5819, "172.29.1.1", 5809);

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if (Constants.currentMode == Mode.SIM) {
      robotContainer.resetSimulationField();
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (Constants.currentMode == Mode.SIM) {
      robotContainer.resetSimRobotPose();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true);
    // Swerve Motors
    TalonFX instrument1 = new TalonFX(TunerConstants.FrontLeft.DriveMotorId);
    instrument1.getConfigurator().apply(audioConfigs);
    TalonFX instrument2 = new TalonFX(TunerConstants.FrontLeft.SteerMotorId);
    instrument2.getConfigurator().apply(audioConfigs);
    TalonFX instrument3 = new TalonFX(TunerConstants.FrontRight.DriveMotorId);
    instrument3.getConfigurator().apply(audioConfigs);
    TalonFX instrument4 = new TalonFX(TunerConstants.FrontRight.SteerMotorId);
    instrument4.getConfigurator().apply(audioConfigs);
    TalonFX instrument5 = new TalonFX(TunerConstants.BackLeft.DriveMotorId);
    instrument5.getConfigurator().apply(audioConfigs);
    TalonFX instrument6 = new TalonFX(TunerConstants.BackLeft.SteerMotorId);
    instrument6.getConfigurator().apply(audioConfigs);
    TalonFX instrument7 = new TalonFX(TunerConstants.BackRight.DriveMotorId);
    instrument7.getConfigurator().apply(audioConfigs);
    TalonFX instrument8 = new TalonFX(TunerConstants.BackRight.SteerMotorId);
    instrument8.getConfigurator().apply(audioConfigs);

    // Motors from 2025 bot
    TalonFX intrument9 = new TalonFX(40); // Climb
    intrument9.getConfigurator().apply(audioConfigs);
    TalonFX intrument10 = new TalonFX(42); // Collector
    intrument10.getConfigurator().apply(audioConfigs);
    TalonFX intrument11 = new TalonFX(11); // Collector
    intrument11.getConfigurator().apply(audioConfigs);
    TalonFX intrument12 = new TalonFX(27); // Elevator
    intrument12.getConfigurator().apply(audioConfigs);
    TalonFX intrument13 = new TalonFX(28); // Elevator
    intrument13.getConfigurator().apply(audioConfigs);
    TalonFX intrument14 = new TalonFX(4); // Wrist
    intrument14.getConfigurator().apply(audioConfigs);

    Orchestra orchestra = new Orchestra();

    orchestra.addInstrument(instrument1);
    orchestra.addInstrument(instrument2);
    orchestra.addInstrument(instrument3);
    orchestra.addInstrument(instrument4);
    orchestra.addInstrument(instrument5);
    orchestra.addInstrument(instrument6);
    orchestra.addInstrument(instrument7);
    orchestra.addInstrument(instrument8);
    orchestra.addInstrument(intrument9);
    orchestra.addInstrument(intrument10);
    orchestra.addInstrument(intrument11);
    orchestra.addInstrument(intrument12);
    orchestra.addInstrument(intrument13);
    orchestra.addInstrument(intrument14);
    var status = orchestra.loadMusic("jinglebells.chrp");
    if (status.isOK()) {
      orchestra.play();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }
}
