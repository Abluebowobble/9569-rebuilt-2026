
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
    RUN(0.7),
    REVERSE(-0.9),
    UNJAM(-0.3);

    private final double percentageOutput;

    private Speed(double percentageOutput) {
      this.percentageOutput = percentageOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentageOutput * 12.0);
    }
  }

  public FeederSubsystem(BooleanSupplier shouldFeed) {
    this.shouldFeed = shouldFeed;
    setDefaultCommand(idle());
  }

  /** sets speed based on percentage output given Speed enum */
  public void set(Speed speed) {
    // targetRPM = speed.rpm();
    // controller.setSetpoint(speed.rpm().magnitude(), ControlType.kVelocity);
    motor.setVoltage(speed.voltage());
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

  public Command unJamCommand() {
    return run(() -> set(Speed.REVERSE)).withTimeout(0.25).andThen(runOnce(() -> set(Speed.RUN)));
  }

  /** given parameter 0-1, sets percentage output of motor */
  public void setPercentageOutput(double percentageOutput) {
    SmartDashboard.putNumber("feeder output percentage", percentageOutput);
    motor.setVoltage(percentageOutput * 12.0);
  }

  public Command runWithOverrideCommand(BooleanSupplier reverseButton) {
    return run (() -> {
      if (reverseButton.getAsBoolean()) {
        set(Speed.REVERSE);
      }
      set(Speed.RUN);
    });
  }

  /** set to forward speed enum on start, stop on end */
  public Command runCommand() {
    return run(() -> {
      setState(FeederState.RUNNING);

      if (reversingPulse) {
        set(Speed.REVERSE);

        if (reverseTimer.hasElapsed(0.25)) {
          reversingPulse = false;
          reverseTimer.stop();
          reverseTimer.reset();
        }

        return;
      }

      if (!shouldFeed.getAsBoolean()) {
        reversingPulse = true;
        reverseTimer.restart();
        set(Speed.REVERSE);
        return;
      }

      set(Speed.RUN);
    }).finallyDo(() -> {
      reversingPulse = false;
      reverseTimer.stop();
      reverseTimer.reset();
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

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addBooleanProperty("should feed", shouldFeed, null);
    sendableBuilder.addStringProperty("Command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    sendableBuilder.addDoubleProperty("Supply Current", () -> motor.getOutputCurrent(), null);
    sendableBuilder.addDoubleProperty("RPM", () -> motor.getEncoder().getVelocity(), null);
  }
}
