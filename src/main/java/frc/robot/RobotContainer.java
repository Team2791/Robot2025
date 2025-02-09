package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.NavX;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.util.AdvantageUtil;
import frc.robotreplay.drivetrain.GyroReplay;
import frc.robotreplay.drivetrain.SwerveReplay;
import frc.robotsim.drivetrain.GyroSim;
import frc.robotsim.drivetrain.SwerveSim;

public class RobotContainer {
	// controllers
	final CommandXboxController driverctl;

	// subsystems
	final Drivetrain drivetrain;


	public RobotContainer() {
		this.driverctl = new CommandXboxController(IOConstants.Controller.kDriver);
		this.drivetrain = new Drivetrain(
			AdvantageUtil.matchReal(NavX::new, GyroSim::new, GyroReplay::new),
			AdvantageUtil.matchReal(SwerveModule::new, SwerveSim::new, SwerveReplay::new)
		);

		configureBindings();
	}

	private void configureBindings() {
		this.drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));

		/* Debug command, to be disabled during comps, etc */
		driverctl.a().onTrue(this.drivetrain.characterize());
	}

	public Command getAutonomousCommand() {
		// TODO: autos
		return null;
	}
}
