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
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AdvantageConstants;
import frc.robot.constants.BuildConstants;
import frc.robot.util.Timestamped;

public class Robot extends LoggedRobot {
	final RobotContainer container;

	Command autoCommand;

	public Robot() {
		// setup logger constants
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitStatus", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitStatus", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitStatus", "Unknown");
				break;
		}

		// setup logger data receivers
		switch (AdvantageConstants.Modes.kCurrent) {
			case Real:
				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());

				break;
			case Sim:
				Logger.addDataReceiver(new NT4Publisher());
				break;
			default:
				String logfile = LogFileUtil.findReplayLog();

				setUseTiming(false);
				Logger.setReplaySource(new WPILOGReader(logfile));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logfile, "sim")));
				break;
		}

		// register rev hardware logger
		Logger.registerURCL(URCL.startExternal());

		// start logger
		Logger.start();

		this.container = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		// Performance: give ourselves very high priority
		Threads.setCurrentThreadPriority(true, 99);

		// Run the robot for a tick
		CommandScheduler.getInstance().run();

		// High prio no longer needed
		Threads.setCurrentThreadPriority(false, 10);
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		autoCommand = container.getAutonomousCommand();

		if (autoCommand != null) {
			autoCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}
}
