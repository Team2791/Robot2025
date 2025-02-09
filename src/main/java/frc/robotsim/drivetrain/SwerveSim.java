package frc.robotsim.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.ModuleConstants;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class SwerveSim extends SwerveModule {
	final DCMotorSim driveSim;
	final DCMotorSim turnSim;

	final SparkMaxSim driveSpark;
	final SparkMaxSim turnSpark;

	final DCMotor driveGearbox;
	final DCMotor turnGearbox;

	final Timer timer = new Timer();

	public SwerveSim(int driveId, int turnId, double angularOffset) {
		super(driveId, turnId, angularOffset);

		driveGearbox = DCMotor.getNEO(1);
		turnGearbox = DCMotor.getNeo550(1);

		driveSpark = new SparkMaxSim(driveMotor, driveGearbox);
		turnSpark = new SparkMaxSim(turnMotor, turnGearbox);

		driveSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				driveGearbox,
				ModuleConstants.DriveMotor.kMoI,
				ModuleConstants.DriveMotor.kReduction
			),
			driveGearbox
		);

		turnSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				turnGearbox,
				ModuleConstants.TurnMotor.kMoI,
				ModuleConstants.TurnMotor.kReduction
			),
			turnGearbox
		);

		timer.start();
	}

	@Override
	public void update() {
		// update wpi sim motors
		driveSim.setInput(driveSpark.getAppliedOutput() * RobotController.getBatteryVoltage());
		turnSim.setInput(turnSpark.getAppliedOutput() * RobotController.getBatteryVoltage());

		driveSim.update(0.02);
		turnSim.update(0.02);

		// update spark
		driveSpark.iterate(
			driveSim.getAngularVelocityRPM() * ModuleConstants.DriveEncoder.kVelocityFactor,
			RoboRioSim.getVInVoltage(),
			0.02
		);

		turnSpark.iterate(
			turnSim.getAngularVelocityRPM() * ModuleConstants.TurnEncoder.kVelocityFactor,
			RoboRioSim.getVInVoltage(),
			0.02
		);

		// actually update the inputs
		data.driveConnected = true;
		data.drivePosition = Meters.of(
			driveSim.getAngularPositionRotations() * ModuleConstants.DriveEncoder.kPositionFactor
		);
		data.driveVelocity = MetersPerSecond.of(
			driveSim.getAngularVelocityRPM() * ModuleConstants.DriveEncoder.kVelocityFactor
		);
		data.driveVoltage = Volts.of(driveSim.getInputVoltage());
		data.driveCurrent = Amps.of(driveSim.getCurrentDrawAmps());

		data.turnConnected = true;
		data.turnPosition = Radians.of(
			turnSim.getAngularPositionRotations() * ModuleConstants.TurnEncoder.kPositionFactor
		);
		data.turnVelocity = RadiansPerSecond.of(
			turnSim.getAngularVelocityRPM() * ModuleConstants.TurnEncoder.kVelocityFactor
		);
		data.turnVoltage = Volts.of(driveSim.getInputVoltage());
		data.turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());
	}

	public void setDesiredState(SwerveModuleState desired) {
		super.setDesiredState(desired);
	}
}
