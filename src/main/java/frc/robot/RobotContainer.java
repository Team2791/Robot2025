package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.NavX;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.util.AdvantageUtil;
import frc.robotreplay.drivetrain.GyroReplay;
import frc.robotreplay.drivetrain.SwerveReplay;
import frc.robotsim.drivetrain.SwerveSim;

public class RobotContainer {
	// controllers
	final CommandXboxController driverctl = new CommandXboxController(0);

	// subsystems
	final Drivetrain drivetrain = new Drivetrain(
		AdvantageUtil.getReal(
			NavX::new,
			GyroReplay::new,
			GyroReplay::new
		),
		AdvantageUtil.matchReal(
			SwerveModule::new,
			SwerveSim::new,
			SwerveReplay::new
		)
	);

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		this.drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
	}

	public Command getAutonomousCommand() {
		// TODO: autos
		return null;
	}
}
