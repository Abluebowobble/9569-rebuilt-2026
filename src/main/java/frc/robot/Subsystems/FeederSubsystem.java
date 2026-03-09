
package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class FeederSubsystem extends SubsystemBase {
  private final SparkMax motor = new SparkMax(HardwareMap.FEEDER, MotorType.kBrushless);

  public enum Speed {
    STOP(0),
    RUN(0.7); // to tune

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12.0);
    }
  }

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig();
    motor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData(this);

  }

  public void set(Speed speed) {
    motor.setVoltage(speed.voltage());
  }

  public void setPercentageOutput(double percentage) {
    motor.setVoltage(percentage * motor.getBusVoltage());
  }

  public Command runCommand() {
    return startEnd(() -> set(Speed.RUN), () -> set(Speed.STOP));
  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    sendableBuilder.addDoubleProperty("Supply Current", () -> motor.getOutputCurrent(), null);
  }
}
