package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoManager;
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

	// autos
	final AutoManager autos = new AutoManager(drivetrain);
	final AutoChooser chooser = new AutoChooser();

	public RobotContainer() throws IOException, ParseException {
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
