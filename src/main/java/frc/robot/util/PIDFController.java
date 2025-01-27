package frc.robot.util;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDFController extends PIDController {
	private double kF;
	private double lastOutput;
	private double minOutput;
	private double maxOutput;

	public PIDFController(
		double kP,
		double kI,
		double kD,
		double kF
	) {
		super(kP, kI, kD);
		this.kF = kF;
		this.lastOutput = Double.NaN;
		this.minOutput = Double.NEGATIVE_INFINITY;
		this.maxOutput = Double.POSITIVE_INFINITY;
	}

	public void setOutputRange(double minOutput, double maxOutput) {
		this.minOutput = minOutput;
		this.maxOutput = maxOutput;
	}

	public double getF() { return kF; }

	public void setF(double kF) { this.kF = kF; }

	@Override
	public double calculate(double measurement, double setpoint) {
		this.lastOutput = super.calculate(measurement, setpoint) + this.kF * setpoint;
		return this.clampOutput();
	}

	@Override
	public double calculate(double measurement) {
		this.lastOutput = super.calculate(measurement) + this.kF * getSetpoint();
		return this.clampOutput();
	}

	private double clampOutput() {
		this.lastOutput = Math.min(Math.max(this.lastOutput, this.minOutput), this.maxOutput);
		return this.lastOutput;
	}

	public double getLastOutput() { return this.lastOutput; }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty("f", this::getF, this::setF);
	}
}
