package frc.robotio.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public abstract class ElevatorIO {
	@AutoLog
	public static class ElevatorData {
		public boolean leaderConnected = false;
		public Voltage leaderVoltage = Volts.of(0);
		public Current leaderCurrent = Amps.of(0);

		public boolean followerConnected;
		public Voltage followerVoltage = Volts.of(0);
		public Current followerCurrent = Amps.of(0);

		public Angle position;
		public AngularVelocity velocity;

		public double[] timestamps = new double[0];
		public double[] positions = new double[0];
	}

	public final ElevatorDataAutoLogged data = new ElevatorDataAutoLogged();

	public abstract void update();

	public abstract void setDesiredPosition(Angle position);
}
