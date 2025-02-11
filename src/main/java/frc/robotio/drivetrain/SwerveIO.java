package frc.robotio.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public abstract class SwerveIO {
	@AutoLog
	public static class SwerveData {
		public boolean driveConnected = false;
		public Distance drivePosition = Meters.of(0); // wheel position
		public LinearVelocity driveVelocity = MetersPerSecond.of(0); // wheel velocity
		public Voltage driveVoltage = Volts.of(0);
		public Current driveCurrent = Amps.of(0);

		public boolean turnConnected = false;
		public Angle turnPosition = Radians.of(0);
		public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
		public Voltage turnVoltage = Volts.of(0);
		public Current turnCurrent = Amps.of(0);

		public SwerveModuleState desired = new SwerveModuleState();
		public SwerveModuleState corrected = new SwerveModuleState();

		public double[] timestamps = new double[0];
		public double[] drivePositions = new double[0];
		public double[] turnPositions = new double[0];
	}

	public final SwerveDataAutoLogged data = new SwerveDataAutoLogged();

	protected final double angularOffset;
	protected final double driveId;
	protected final double turnId;

	public SwerveIO(
		int driveId,
		int turnId,
		double angularOffset
	) {
		this.driveId = driveId;
		this.turnId = turnId;
		this.angularOffset = angularOffset;
	}

	public abstract void update();

	public abstract void setDesiredState(SwerveModuleState desired);

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			data.drivePosition,
			new Rotation2d(data.turnPosition.minus(Radians.of(angularOffset)))
		);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			data.driveVelocity,
			new Rotation2d(data.turnPosition.minus(Radians.of(angularOffset)))
		);
	}
}
