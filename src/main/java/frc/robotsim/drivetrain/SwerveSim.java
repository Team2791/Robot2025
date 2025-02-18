package frc.robotsim.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.PIDConstants;
import frc.robot.util.PIDFController;
import frc.robotio.drivetrain.SwerveIO;

public class SwerveSim extends SwerveIO {
	final DCMotorSim driveSim;
	final DCMotorSim turnSim;

	final PIDFController drivectl;
	final PIDFController turnctl;

	public SwerveSim(int driveId, int turnId, double angularOffset) {
		super(driveId, turnId, angularOffset);

		driveSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(1),
				ModuleConstants.DriveMotor.kMoI,
				ModuleConstants.DriveMotor.kReduction
			),
			DCMotor.getNEO(1)
		);

		turnSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNeo550(1),
				ModuleConstants.TurnMotor.kMoI,
				ModuleConstants.TurnMotor.kReduction
			),
			DCMotor.getNeo550(1)
		);

		drivectl = new PIDFController(
			PIDConstants.DriveMotor.kP,
			PIDConstants.DriveMotor.kI,
			PIDConstants.DriveMotor.kD,
			PIDConstants.DriveMotor.kF
		);

		turnctl = new PIDFController(
			PIDConstants.TurnMotor.kP,
			PIDConstants.TurnMotor.kI,
			PIDConstants.TurnMotor.kD,
			PIDConstants.TurnMotor.kF
		);

		drivectl.setOutputRange(PIDConstants.DriveMotor.kMin, PIDConstants.DriveMotor.kMax);
		turnctl.setOutputRange(PIDConstants.TurnMotor.kMin, PIDConstants.TurnMotor.kMax);
		turnctl.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void update() {
		turnctl.setSetpoint(Degrees.of(90).in(Radians));

		// do a drive
		final double drivePow = drivectl.calculate(
			driveSim.getAngularVelocityRPM() * ModuleConstants.DriveEncoder.kVelocityFactor
		);
		final double turnPow = turnctl.calculate(
			turnSim.getAngularPositionRotations() * ModuleConstants.TurnEncoder.kPositionFactor
		);

		// https://www.chiefdelphi.com/t/sparkmax-set-vs-setvoltage/415059/2
		// correct for differing voltages cuz battery won't always be 12V
		final double driveVolts = drivePow * RobotController.getBatteryVoltage();
		final double turnVolts = turnPow * RobotController.getBatteryVoltage();

		// set the input voltage to simulate
		driveSim.setInputVoltage(driveVolts);
		turnSim.setInputVoltage(turnVolts);

		driveSim.update(0.02);
		turnSim.update(0.02);

		// actually update the inputs
		data.driveConnected = true;
		data.drivePosition = Meters.of(
			driveSim.getAngularPositionRotations() * ModuleConstants.DriveEncoder.kPositionFactor
		);
		data.driveVelocity = MetersPerSecond.of(
			driveSim.getAngularVelocityRPM() * ModuleConstants.DriveEncoder.kVelocityFactor
		);
		data.driveVoltage = Volts.of(driveVolts);
		data.driveCurrent = Amps.of(driveSim.getCurrentDrawAmps());

		data.turnConnected = true;
		data.turnPosition = Radians.of(
			turnSim.getAngularPositionRotations() * ModuleConstants.TurnEncoder.kPositionFactor
		);
		data.turnVelocity = RadiansPerSecond.of(
			turnSim.getAngularVelocityRPM() * ModuleConstants.TurnEncoder.kVelocityFactor
		);
		data.turnVoltage = Volts.of(turnVolts);
		data.turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());
	}

	public void setDesiredState(SwerveModuleState desired) {
		SwerveModuleState corrected = new SwerveModuleState(
			desired.speedMetersPerSecond,
			desired.angle.plus(Rotation2d.fromRadians(this.data.angularOffset))
		);

		corrected.optimize(new Rotation2d(turnSim.getAngularPositionRad()));

		drivectl.setSetpoint(corrected.speedMetersPerSecond);
		turnctl.setSetpoint(corrected.angle.getRadians());

		this.data.desired = desired;
		this.data.corrected = corrected;
	}
}
