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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {

  private final SparkMax motor = new SparkMax(HardwareMap.CONVEYOR, MotorType.kBrushless);
  private Time lastUpdateTime = Seconds.of(Timer.getFPGATimestamp());
  private Voltage curVoltage = Volts.of(0);

  private final static Voltage kVoltageSlew = Volts.of(0.1);

  public enum Speed {
    STOP(0),
    RUN(0.8), // to tune
    REVERSE(-0.5); // to tune 

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12.0);
    }
  }

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig(); 
    motor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** sets speed based on speed enum in percentage output */
  public void set(Speed speed) {
    motor.setVoltage(speed.voltage());
  }

  public void updateVoltage() {

  }

  /** set speed given a percentage output */
  public void setPercentageOutput(double percentage) {
    motor.setVoltage(percentage * motor.getBusVoltage());
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
  public void periodic() {
    updateVoltage();
    SmartDashboard.putNumber("current output", motor.getOutputCurrent());
  }
}
