// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import frc.robot.Commands.GeneralRobotCommands.ConveyorState;
import frc.robot.Commands.GeneralRobotCommands.FeederState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {

  private final SparkMax motor = new SparkMax(HardwareMap.CONVEYOR, MotorType.kBrushless);

  private ConveyorState conveyorState = ConveyorState.STOP;

  public enum Speed {
    STOP(0),
    RUN(0.3), // to tune
    REVERSE(-0.9); // to tune

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12);
    }
  }

  public ConveyorSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(90, 60);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setDefaultCommand(idle());
  }

  /** sets speed based on speed enum in percentage output */
  public void set(Speed speed) {
    motor.setVoltage(speed.voltage());
  }

  /** set speed given a percentage output */
  public void setPercentageOutput(double percentage) {
    SmartDashboard.putNumber("conveyor output percentage", percentage);
    motor.setVoltage(percentage * 12.0);
  }

  /** runs the conveyor, stops on end */
  public Command runCommand() {
    return runOnce(() -> set(Speed.RUN)).alongWith(Commands.runOnce(() -> setState(ConveyorState.RUNNING)));
  }

  @Override
  public Command idle() {
    return runOnce(() -> set(Speed.STOP)).alongWith(Commands.runOnce(() -> setState(ConveyorState.STOP)));
  }

  public void setState(ConveyorState conveyorState) {
    this.conveyorState = conveyorState;
  }

  public ConveyorState getState() {
    return conveyorState;
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

  /** reverse the conveyor, stops on end */
  public Command reverseCommand() {
    return run(() -> set(Speed.REVERSE)).alongWith(Commands.runOnce(() -> setState(ConveyorState.REVERSE)));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current output", () -> motor.getOutputCurrent(), null);

  }
}
