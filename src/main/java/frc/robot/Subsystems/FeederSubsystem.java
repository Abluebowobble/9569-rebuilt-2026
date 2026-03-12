
package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  
  private final SparkMax motor = new SparkMax(HardwareMap.FEEDER, MotorType.kBrushless);

  public enum Speed {
    STOP(0),
    RUN(0.7),
    REVERSE(-0.7);

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
  }

  /** sets speed based on percentage output given Speed enum */
  public void set(Speed speed) {
    motor.setVoltage(speed.voltage());
  }

  /** sets speed given Voltage volts */
  public void set(Voltage volts) {
    motor.setVoltage(volts);
  }

  /** given parameter 0-1, sets percentage output of motor */
  public void setPercentageOutput(double percentage) {
    motor.setVoltage(percentage * motor.getBusVoltage());
  }

  /** set to forward speed enum on start, stop on end */
  public Command runCommand() {
    return startEnd(() -> set(Speed.RUN), () -> set(Speed.STOP));
  }

  /** set to reverse speed enum on start, stop on end*/
  public Command reverseCommand() {
    return startEnd(() -> set(Speed.REVERSE), () -> set(Speed.STOP));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addStringProperty("Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    sendableBuilder.addDoubleProperty("Supply Current", () -> motor.getOutputCurrent(), null);
  }
}
