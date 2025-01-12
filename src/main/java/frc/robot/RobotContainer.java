package frc.robot;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
	// controllers
	final CommandXboxController driverctl = new CommandXboxController(0);

	// subsystems
	final Drivetrain drivetrain = new Drivetrain();

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
