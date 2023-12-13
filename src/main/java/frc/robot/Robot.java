// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	private Command autonomousCommand;

	private RobotContainer robotContainer;

	private PowerDistribution pdh;

	@Override
	public void robotInit() {
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

		Logger.recordMetadata("ProjectName", "2024-Mushussu"); // Set a metadata value

		switch (Constants.currentMode) {
			case REAL:
				// Running on a real robot, log to a USB stick
				Logger.addDataReceiver(new WPILOGWriter("/U"));
				Logger.addDataReceiver(new NT4Publisher());
				pdh = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
				pdh.setSwitchableChannel(false);
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

		// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
		// the "Understanding Data Flow" page
		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
						// be added.

		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		if(Constants.currentMode == Constants.Mode.REAL)
			pdh.setSwitchableChannel(false);
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		robotContainer.enabledInit();
		if(Constants.currentMode == Constants.Mode.REAL)
			pdh.setSwitchableChannel(true);
	}

	@Override
	public void autonomousInit() {
		// m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		autonomousCommand = new PathPlannerAuto("Test Auto");

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
