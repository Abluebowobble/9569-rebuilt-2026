// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

  // Motors
  private final SparkMax pivotMotor = new SparkMax(HardwareMap.INTAKE_PIVOT, MotorType.kBrushless);
  private final SparkMax rollerMotor = new SparkMax(HardwareMap.INTAKE_ROLLER, MotorType.kBrushless);

  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  private Position setPointAngle;
  private static final Angle kPositionTolerance = Degrees.of(0); // to tune

  // pivot motor controller
  private final PIDController pivotMotorController = new PIDController(0, 0, 0); // to tune

  // speed and position enums
  public enum Speed {
    STOP(0),
    INTAKE(0.75); // to tune

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12.0);
    }
  }

  public enum Position {
    // all of this is to tune
    STOWED(0),
    INTAKE(0),
    AGITATE(0);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle degrees() {
      return Degrees.of(degrees);
    }
  }

  public IntakeSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig(); // might absstract to parent class?
    pivotMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder.setPosition(0);

    SmartDashboard.putData(this);
  }

  public void set(Speed speed) {
    pivotMotor.setVoltage(speed.voltage());
  }

  public void set(Position position) {
    setPointAngle = position;
    double output = pivotMotorController.calculate(pivotEncoder.getPosition(),
        (position.degrees().baseUnitMagnitude() % 360) / 360);
    pivotMotor.setVoltage(output * pivotMotor.getBusVoltage());
  }

  public Command intakeCommand() {
    return startEnd(
        () -> {
          set(Position.INTAKE);
          set(Speed.INTAKE);
        },
        () -> set(Speed.STOP));
  }

  public boolean isPositionWithinTolerance() {
    final Angle cur = Degrees.of(pivotEncoder.getPosition() * 360);
    final Angle target = setPointAngle.degrees();

    return cur.isNear(target, kPositionTolerance);
  }

  public Command agitateCommand() {
    return runOnce(() -> set(Speed.INTAKE))
        .andThen(
            Commands.sequence(
                runOnce(() -> set(Position.AGITATE)),
                Commands.waitUntil(this::isPositionWithinTolerance),
                runOnce(() -> set(Position.INTAKE)),
                Commands.waitUntil(this::isPositionWithinTolerance))
                .repeatedly())
        .handleInterrupt(() -> {
          set(Position.INTAKE);
          set(Speed.STOP);
        });
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty("Angle (degrees)", () -> pivotEncoder.getPosition(), null);
    builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getOutputCurrent(), null);
    
  }
}
