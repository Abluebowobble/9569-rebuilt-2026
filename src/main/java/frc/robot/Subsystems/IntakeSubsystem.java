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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // electronics
  private final SparkMax pivotMotor = new SparkMax(HardwareMap.INTAKE_PIVOT, MotorType.kBrushless);
  private final SparkMax rollerMotor = new SparkMax(HardwareMap.INTAKE_ROLLER, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  // set point for pid
  private Angle setPointAngle = Degrees.of(0);

  // range of allowed positions
  private static final Angle kPositionTolerance = Degrees.of(5); // to tune

  // gear reduction
  private final Angle kDegreesPerRotation = Degrees.of(2);

  private final SparkClosedLoopController controller = pivotMotor.getClosedLoopController();

  // speed for roller motor
  public enum Speed {
    STOP(0),
    INTAKE(0.9), // to tune
    REVERSE(-0.9); // to tune

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12.0);
    }
  }

  // set angle for pivot motor
  public enum Position {
    STOWED(Degrees.of(8)),
    INTAKE(Degrees.of(83.7)),
    AGITATEUP(Degrees.of(60)),
    AGITATEDOWN(Degrees.of(70));

    private final Angle degrees;

    private Position(Angle degrees) {
      this.degrees = degrees;
    }

    public Angle degrees() {
      return degrees;
    }
  }

  public IntakeSubsystem() {
    SparkBaseConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.closedLoop.p(0.7);
    pivotConfig.smartCurrentLimit(80);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder.setPosition(0);
    setDefaultCommand(idle());
  }

  public Command zeroCommand() {
    return Commands.runOnce(() -> pivotEncoder.setPosition(0));
  }

  /** set roller to percentage output given speed enum */
  public void set(Speed speed) {
    rollerMotor.setVoltage(speed.voltage());
  }

  /** set pivot motor to position given Position enum */
  public void set(Position position) {
    setPointAngle = position.degrees();
    controller.setSetpoint(position.degrees().div(kDegreesPerRotation).magnitude(), ControlType.kPosition);
  }

  /** set pivot motor to go to intake position */
  public Command intakePositionCommand() {
    return runOnce(() -> set(Position.INTAKE));
  }

  /** Return to home position */
  public Command returnPositionCommand() {
    return runOnce(() -> set(Position.STOWED));
  }

  /**
   * humps balls in basket, goes back to intake position and stops on interrupt
   */
  public Command agitatePivotCommand() {
    return runOnce(() -> set(Speed.INTAKE))
        .andThen(
            Commands.sequence(
                runOnce(() -> set(Position.AGITATEUP)),
                Commands.waitUntil(this::isPositionWithinTolerance),
                runOnce(() -> set(Position.AGITATEDOWN)),
                Commands.waitUntil(this::isPositionWithinTolerance))
                .repeatedly())
        .handleInterrupt(() -> {
          set(Position.INTAKE);
          set(Speed.STOP);
        }).onlyIf(() -> !isStowed());
  }

  public boolean isStowed() {
    return setPointAngle.isNear(Position.STOWED.degrees(), kPositionTolerance);
  }

  public Command runRollerCommand() {
    return run(() -> set(Speed.INTAKE));
  }

  public Command stopRollerCommand() {
    return runOnce(() -> set(Speed.STOP));
  }

  public Command reverseRollerCommand() {
    return run(() -> set(Speed.REVERSE));
  }

  @Override
  public Command idle() {
    return runOnce(() -> set(Speed.STOP));
  }

  /** checks if angle of pivot is within kPositionTolerance */
  public boolean isPositionWithinTolerance() {
    final Angle cur = Degrees.of(pivotEncoder.getPosition() * kDegreesPerRotation.magnitude());
    final Angle target = setPointAngle;

    return cur.isNear(target, kPositionTolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    // updatePivotPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position (rotations)", () -> pivotEncoder.getPosition(), null);
    builder.addDoubleProperty("output (current)", () -> pivotMotor.getOutputCurrent(), null);
  }

  // /** updates pivot position with pid, to add: slew */
  // public void updatePivotPosition() {

  // final double currentPosition = pivotEncoder.getPosition();
  // final double targetPosition = setPointAngle.magnitude() /
  // kDegreesPerRotation;

  // // get new voltage according to pid controller
  // double pidOutput = pivotMotorController.calculate(currentPosition,
  // targetPosition);

  // // tuned pid for voltage
  // pivotMotor.setVoltage(pidOutput);
  // }

}
