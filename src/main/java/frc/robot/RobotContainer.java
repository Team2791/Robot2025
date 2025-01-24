package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.NavX;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.TriSupplier;
import frc.robotio.drivetrain.GyroIO;
import frc.robotio.drivetrain.SwerveIO;

public class RobotContainer {
	// controllers
	final CommandXboxController driverctl = new CommandXboxController(0);

	// subsystems
	final Drivetrain drivetrain = new Drivetrain(
		AdvantageUtil.<Supplier<GyroIO>>matchReal(
			NavX::new,
			() -> null,
			() -> null
		).get(),
		AdvantageUtil.<TriSupplier<Integer, Integer, Angle, SwerveIO>>matchReal(
			SwerveModule::new,
			(_a, _b, _c) -> null,
			(_a, _b, _c) -> null
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
