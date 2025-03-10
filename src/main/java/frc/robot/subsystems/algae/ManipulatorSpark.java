package frc.robot.subsystems.algae;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.AlgaeManipulatorConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.SparkConfigConstants;

import static edu.wpi.first.units.Units.*;

public class ManipulatorSpark extends ManipulatorIO {
    protected final SparkMax turn;
    protected final SparkMax spin;

    final RelativeEncoder turnEncoder;
    final RelativeEncoder spinEncoder;

    final SparkClosedLoopController turnController;

    public ManipulatorSpark() {
        turn = new SparkMax(IOConstants.AlgaeManipulator.kTurn, MotorType.kBrushless);
        spin = new SparkMax(IOConstants.AlgaeManipulator.kSpin, MotorType.kBrushless);

        turnEncoder = turn.getEncoder();
        spinEncoder = spin.getEncoder();
        turnController = turn.getClosedLoopController();

        turn.configure(
            SparkConfigConstants.AlgaeManipulator.kTurn,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        spin.configure(
            SparkConfigConstants.AlgaeManipulator.kSpin,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        turnEncoder.setPosition(AlgaeManipulatorConstants.Setpoints.kDisabled);
    }

    @Override
    public void update() {
        this.data.turnConnected = turn.getLastError() == REVLibError.kOk;
        this.data.turnVoltage = Volts.of(turn.getBusVoltage() * turn.getAppliedOutput());
        this.data.turnCurrent = Amps.of(turn.getOutputCurrent());
        this.data.turnPosition = Radians.of(turnEncoder.getPosition());

        this.data.spinConnected = spin.getLastError() == REVLibError.kOk;
        this.data.spinVoltage = Volts.of(spin.getBusVoltage() * spin.getAppliedOutput());
        this.data.spinCurrent = Amps.of(spin.getOutputCurrent());
        this.data.spinVelocity = RadiansPerSecond.of(spinEncoder.getVelocity());
    }

    @Override
    protected void set(boolean enabled) {
        if (enabled) {
            turnController.setReference(AlgaeManipulatorConstants.Setpoints.kEnabled, ControlType.kPosition);
            spin.set(AlgaeManipulatorConstants.Setpoints.kPower);
        } else {
            turnController.setReference(AlgaeManipulatorConstants.Setpoints.kDisabled, ControlType.kPosition);
            spin.set(0);
        }
    }
}
