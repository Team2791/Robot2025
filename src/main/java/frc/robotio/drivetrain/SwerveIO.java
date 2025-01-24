package frc.robotio.drivetrain;

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
import frc.robot.util.Timestamped;

public abstract class SwerveIO {
	@AutoLog
	public static class SwerveData {
		public boolean driveConnected;
		public Distance drivePosition;
		public LinearVelocity driveVelocity;
		public Voltage driveVoltage;
		public Current driveCurrent;

		public boolean turnConnected;
		public Angle turnPosition;
		public AngularVelocity turnVelocity;
		public Voltage turnVoltage;
		public Current turnCurrent;

		public Timestamped<Distance>[] driveCached;
		public Timestamped<Angle>[] turnCached;
	}

	public final SwerveDataAutoLogged data = new SwerveDataAutoLogged();
	protected final Angle angularOffset;

	public SwerveIO(Angle angularOffset) {
		this.angularOffset = angularOffset;
	}

	public abstract void update();

	public abstract void setDesiredState(SwerveModuleState desired);

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			data.drivePosition,
			new Rotation2d(data.turnPosition.minus(angularOffset))
		);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			data.driveVelocity,
			new Rotation2d(data.turnPosition)
		);
	}
}
