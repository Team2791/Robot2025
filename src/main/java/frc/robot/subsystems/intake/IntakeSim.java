package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakeConstants.Motor;
import frc.robot.constants.SparkConfigConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeSim extends IntakeIO {
	final SparkMax leftMotor;
	final SparkMax rightMotor;

	final SparkMaxSim leftSim;
	final SparkMaxSim rightSim;

	final DCMotorSim leftMechanism;
	final DCMotorSim rightMechanism;

	final RelativeEncoder leftEncoder;
	final RelativeEncoder rightEncoder;

	final SparkLimitSwitchSim beamSim;

	public IntakeSim() {
		DCMotor gearbox = DCMotor.getNEO(2);

		leftMotor = new SparkMax(IOConstants.Roller.kLeft, MotorType.kBrushless);
		rightMotor = new SparkMax(IOConstants.Roller.kRight, MotorType.kBrushless);

		leftEncoder = leftMotor.getEncoder();
		rightEncoder = rightMotor.getEncoder();


		leftMotor.configure(
			SparkConfigConstants.Roller.kLeft,
			SparkConfigConstants.kResetMode,
			SparkConfigConstants.kPersistMode
		);

		leftSim = new SparkMaxSim(leftMotor, gearbox);
		rightSim = new SparkMaxSim(rightMotor, gearbox);
		beamSim = new SparkLimitSwitchSim(leftMotor, false);

		leftMechanism = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(gearbox, Motor.kMoI, Motor.kReduction),
			gearbox
		);
		rightMechanism = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(gearbox, Motor.kMoI, Motor.kReduction),
			gearbox
		);
	}

	@Override
	public void update() {
		// update wpi mech
		leftMechanism.setInputVoltage(leftSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
		leftMechanism.update(0.02);

		// update spark sim
		leftSim.iterate(
			leftMechanism.getAngularVelocityRadPerSec(),
			RoboRioSim.getVInVoltage(),
			0.02
		);

		// account for current draw
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(leftMechanism.getCurrentDrawAmps()));

		// data
		this.data.leftConnected = false;
		this.data.leftVoltage = Volts.of(leftSim.getBusVoltage() * RoboRioSim.getVInVoltage());
		this.data.leftCurrent = Amps.of(leftMechanism.getCurrentDrawAmps());
		this.data.leftVelocity = RadiansPerSecond.of(leftEncoder.getVelocity() * Motor.kReduction);

		this.data.rightConnected = false;
		this.data.rightVoltage = Volts.of(rightSim.getBusVoltage() * RoboRioSim.getVInVoltage());
		this.data.rightCurrent = Amps.of(rightMechanism.getCurrentDrawAmps());
		this.data.rightVelocity = RadiansPerSecond.of(rightEncoder.getVelocity() * Motor.kReduction);

		this.data.broken = beamSim.getPressed();
	}

	@Override
	public void set(double left, double right) {
		leftMotor.set(left);
		rightMotor.set(right);
	}

	void flipBeam() {
		beamSim.setPressed(!beamSim.getPressed());
	}
}
