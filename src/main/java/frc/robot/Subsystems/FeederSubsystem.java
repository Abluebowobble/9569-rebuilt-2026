
package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Commands.GeneralRobotCommands.FeederState;
import frc.robot.Commands.GeneralRobotCommands.HoodState;
import frc.robot.Constants.HardwareMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax motor = new SparkMax(HardwareMap.FEEDER, MotorType.kBrushless);

  private AngularVelocity targetRPM = RPM.of(0);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  private FeederState feederState = FeederState.STOP;

  // public enum Speed {
  // STOP(0),
  // RUN(0.5),
  // REVERSE(-0.9);

  // private final double percentOutput;

  // private Speed(double percentOutput) {
  // this.percentOutput = percentOutput;
  // }

  // public Voltage voltage() {
  // return Volts.of(percentOutput * 12.0);
  // }
  // }
  private final Timer reverseTimer = new Timer();
  private boolean reversingPulse = false;

  public BooleanSupplier shouldFeed;

  public enum Speed {
    STOP(0),
    REVERSE(-0.5),
    RUN(0.93),
    UNJAM(-0.5);

    private final double percentageOutput;

    private Speed(double percentageOutput) {
      this.percentageOutput = percentageOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentageOutput * 12.0);
    }
  }

  private DoubleSupplier distanceToHubSupplier;

  public FeederSubsystem(BooleanSupplier shouldFeed, DoubleSupplier distanceToHubSupplier) {
    SparkBaseConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(70, 60);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.shouldFeed = shouldFeed;
    this.distanceToHubSupplier = distanceToHubSupplier;
    setDefaultCommand(idle());
  }

  /** sets speed based on percentage output given Speed enum */
  public void set(Speed speed) {
    // targetRPM = speed.rpm();
    // controller.setSetpoint(speed.rpm().magnitude(), ControlType.kVelocity);

    motor.setVoltage(speed.voltage());
  }

  public double getFeederSpeed() {
    return -0.05 * distanceToHubSupplier.getAsDouble() + 0.4;
  }

  /** sets speed given Voltage volts */
  public void set(Voltage volts) {
    motor.setVoltage(volts);
  }

  public void setState(FeederState feederState) {
    this.feederState = feederState;
  }

  public FeederState getState() {
    return feederState;
  }

  /** given parameter 0-1, sets percentage output of motor */
  public void set(double percentageOutput) {
    motor.setVoltage(percentageOutput * 12.0);
  }

  public Command runWithOverrideCommand(BooleanSupplier reverseButton) {
    return run(() -> {
      if (reverseButton.getAsBoolean()) {
        set(Speed.REVERSE);
      } else {
        set(Speed.RUN);
      }
    });
  }

  /** set to forward speed enum on start, stop on end */
  public Command runCommand() {
    return run(() -> {
      setState(FeederState.RUNNING);
      set(Speed.RUN);
    });
  }

  @Override
  public Command idle() {
    return run(() -> set(Speed.STOP)).alongWith(Commands.runOnce(() -> setState(FeederState.STOP)));
  }

  /** set to reverse speed enum on start, stop on end */
  public Command reverseCommand() {
    return run(() -> set(Speed.REVERSE))
        .alongWith(Commands.runOnce(() -> setState(FeederState.REVERSE)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
  }

  public String toString() {
    switch (feederState) {
      case RUNNING: return "FEEDING";
      case REVERSE: return "REVERSING";
      case STOP: default: return "STOPPED";
    }
  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addBooleanProperty("should feed", shouldFeed, null);
    sendableBuilder.addStringProperty("Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    sendableBuilder.addDoubleProperty("Feeder Supply Current (A)", () -> motor.getOutputCurrent(), null);
    sendableBuilder.addDoubleProperty("RPM", () -> motor.getEncoder().getVelocity(), null);
    sendableBuilder.addDoubleProperty("time", () -> Timer.getFPGATimestamp(), null);
    sendableBuilder.addStringProperty("Current Indexer State", this::toString, null);
  }
}
