// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import frc.robot.Commands.SubsystemsController.ConveyorState;
import frc.robot.Commands.SubsystemsController.FeederState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {

  private final SparkMax motor = new SparkMax(HardwareMap.CONVEYOR, MotorType.kBrushless);

  private ConveyorState state = ConveyorState.STOP;

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
    setDefaultCommand(run(() -> stateManager()));
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
    return startEnd(() -> set(Speed.RUN), () -> set(Speed.STOP));
  }

  /** reverse the conveyor, stops on end */
  public Command reverseCommand() {
    return startEnd(() -> set(Speed.REVERSE), () -> set(Speed.STOP));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current output", () -> motor.getOutputCurrent(), null);

  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
  }

  public void stateManager() {
    switch (state) {
      case REVERSE:
        runOnce(() -> set(Speed.REVERSE));
      case RUNNING:
        runOnce(() -> set(Speed.REVERSE));
      case STOP:
      default:
        runOnce(() -> set(Speed.REVERSE));
    }
  }

  public void setState(ConveyorState conveyorState) {
    state = conveyorState;
  }
}
