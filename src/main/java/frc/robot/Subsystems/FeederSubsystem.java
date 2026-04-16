
package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.MotorType;

import frc.robot.Commands.GeneralRobotCommands.FeederState;
import frc.robot.Constants.HardwareMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

  private final ThriftyNova motor = new ThriftyNova(HardwareMap.FEEDER, MotorType.NEO);

  private AngularVelocity targetRPM = RPM.of(0);

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
  public BooleanSupplier shouldFeed;

  public enum Speed {
    STOP(0),
    REVERSE(-0.5),
    RUN(0.9),
    UNJAM(-0.5);

    private final double percentageOutput;

    private Speed(double percentageOutput) {
      this.percentageOutput = percentageOutput;
    }

    // public Voltage voltage() {
    //   return Volts.of(percentageOutput * 12.0);
    // }
  }

  private DoubleSupplier distanceToHubSupplier;

  public FeederSubsystem(BooleanSupplier shouldFeed, DoubleSupplier distanceToHubSupplier) {
    motor.factoryReset();
    motor.setMotorType(MotorType.NEO);
    motor.setInversion(false);
    motor.enableHardLimits(false);
    motor.enableSoftLimits(false);
    motor.setMaxOutput(1.0, 1.0);
    motor.setMaxCurrent(CurrentType.SUPPLY, 60);
    this.shouldFeed = shouldFeed;
    this.distanceToHubSupplier = distanceToHubSupplier;
    setDefaultCommand(idle());
  }

  /** sets speed based on percentage output given Speed enum */
  public void set(Speed speed) {
    motor.setPercent(speed.percentageOutput);
  }

  /** sets speed given Voltage volts */
  public void set(Voltage volts) {
    motor.setPercent(volts.magnitude() / 12.0);
  }

  public void setState(FeederState feederState) {
    this.feederState = feederState;
  }

  public FeederState getState() {
    return feederState;
  }

  /** given parameter 0-1, sets percentage output of motor */
  public void set(double percentageOutput) {
    motor.setPercent(percentageOutput);
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
  }

  public String toString() {
    switch (feederState) {
      case RUNNING: return "FEEDING";
      case REVERSE: return "REVERSING";
      case STOP: default: return "STOPPED";
    }
  }
}
