
package frc.robotsim.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.PIDConstants;
import frc.robot.util.PIDFController;
import frc.robotio.drivetrain.SwerveIO;

public class SwerveSim extends SwerveIO {
	final SwerveModuleSimulation moduleSim;
	final SimulatedMotorController.GenericMotorController driveSim;
	final SimulatedMotorController.GenericMotorController turnSim;

	final PIDFController drivectl;
	final PIDFController turnctl;

	public SwerveSim(int driveId, int turnId, double angularOffset, SwerveModuleSimulation moduleSim) {
		super(driveId, turnId, angularOffset);

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

		this.moduleSim = moduleSim;

		driveSim = moduleSim
			.useGenericMotorControllerForDrive()
			.withCurrentLimit(Amps.of(ModuleConstants.Neo.kCurrentLimit));

		turnSim = moduleSim
			.useGenericControllerForSteer()
			.withCurrentLimit(Amps.of(ModuleConstants.Neo550.kCurrentLimit));

		drivectl.setOutputRange(PIDConstants.DriveMotor.kMin, PIDConstants.DriveMotor.kMax);
		turnctl.setOutputRange(PIDConstants.TurnMotor.kMin, PIDConstants.TurnMotor.kMax);
		turnctl.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void update() {
		final double drivePow = drivectl.calculate(this.data.driveVelocity.in(MetersPerSecond));
		final double turnPow = turnctl.calculate(this.data.turnPosition.in(Radians));

		// https://www.chiefdelphi.com/t/sparkmax-set-vs-setvoltage/415059/2
		// correct for differing voltages cuz battery won't always be 12V
		final double driveVolts = drivePow * RobotController.getBatteryVoltage();
		final double turnVolts = turnPow * RobotController.getBatteryVoltage();

		// set the input voltage to simulate
		driveSim.requestVoltage(Volts.of(driveVolts));
		turnSim.requestVoltage(Volts.of(turnVolts));

		final double driveWheel = moduleSim.getDriveWheelFinalPosition().in(Radians);
		final double driveVelRot = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);

		final double drivePos = driveWheel * ModuleConstants.Wheel.kCircumference;
		final double driveVel = driveVelRot * ModuleConstants.Wheel.kCircumference;

		final Angle turnPos = moduleSim.getSteerAbsoluteAngle();
		final AngularVelocity turnVel = moduleSim.getSteerAbsoluteEncoderSpeed();

		// actually update the inputs
		data.driveConnected = true;
		data.drivePosition = Meters.of(drivePos);
		data.driveVelocity = MetersPerSecond.of(driveVel);
		data.driveVoltage = Volts.of(driveVolts);
		data.driveCurrent = moduleSim.getDriveMotorSupplyCurrent();

		data.turnConnected = true;
		data.turnPosition = turnPos;
		data.turnVelocity = turnVel;
		data.turnVoltage = Volts.of(turnVolts);
		data.turnCurrent = moduleSim.getSteerMotorSupplyCurrent();
	}

	public void setDesiredState(SwerveModuleState desired) {
		desired.optimize(new Rotation2d(moduleSim.getSteerAbsoluteAngle()));

		drivectl.setSetpoint(desired.speedMetersPerSecond);
		turnctl.setSetpoint(desired.angle.getRadians());

		this.data.desired = desired;
		this.data.corrected = desired;
	}
}