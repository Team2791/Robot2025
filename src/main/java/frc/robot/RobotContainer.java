package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IOConstants;
import frc.robot.maple.MapleSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.NavX;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.util.AdvantageUtil;
import frc.robotreplay.drivetrain.GyroReplay;
import frc.robotreplay.drivetrain.SwerveReplay;

public class RobotContainer {
	// controllers
	final CommandXboxController driverctl;

	// subsystems
	final Drivetrain drivetrain;

	// autos
	final SendableChooser<Command> autoChooser;

	public RobotContainer() throws IOException, ParseException {
		this.driverctl = new CommandXboxController(IOConstants.Controller.kDriver);

		this.drivetrain = new Drivetrain(
			AdvantageUtil.matchReal(NavX::new, MapleSim.getInstance()::makeGyro, GyroReplay::new),
			AdvantageUtil.matchReal(SwerveModule::new, MapleSim.getInstance()::makeModule, SwerveReplay::new)
		);
		this.autoChooser = AutoBuilder.buildAutoChooser();

		SmartDashboard.putData(autoChooser);

		configureBindings();
	}

	private void configureBindings() {
		this.drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
	}

	public Command getAutonomousCommand() { return autoChooser.getSelected(); }
}
