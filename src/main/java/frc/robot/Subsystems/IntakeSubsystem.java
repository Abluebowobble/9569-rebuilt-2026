// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
  private Angle setPointAngle;

  // range of allowed positions
  private static final Angle kPositionTolerance = Degrees.of(1.5); // to tune

  // pivot motor controller
  private final PIDController pivotMotorController = new PIDController(0.45, 0, 0); // to tune

  // for slew
  private static final double kMaxPivotVoltageChange = 0.07;
  private Time lastUpdateTime = Seconds.of(Timer.getFPGATimestamp());
  private Voltage currentOutput = Volts.of(0);

  // gear reduction
  private double degreesPerRotation = 2;

  // speed for roller motor
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

  // set angle for pivot motor
  public enum Position {
    STOWED(8),
    INTAKE(83.7),
    AGITATE(20);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle degrees() {
      return Degrees.of(degrees);
    }
  }

  public IntakeSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig();
    pivotMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder.setPosition(0);
  }

  /** set roller to percentage output given speed enum */
  public void set(Speed speed) {
    pivotMotor.setVoltage(speed.voltage());
  }

  /** set pivot motor to position given Position enum */
  public void set(Position position) {
    setPointAngle = position.degrees().div(degreesPerRotation);
  }

  /** set pivot motor to go to intake position */
  public Command intakePositionCommand() {
    return runOnce(
        () -> set(Position.INTAKE));
  }

  /** Return to home position */
  public Command returnPositionCommand() {
    return runOnce(
        () -> set(Position.STOWED));
  }

  /**
   * humps balls in basket, goes back to intake position and stops on interrupt
   */
  public Command agitatePivotCommand() {
    return Commands.sequence(
        runOnce(() -> set(Position.AGITATE)),
        Commands.waitUntil(this::isPositionWithinTolerance),
        runOnce(() -> set(Position.INTAKE)),
        Commands.waitUntil(this::isPositionWithinTolerance))
        .repeatedly()
        .handleInterrupt(() -> {
          set(Position.INTAKE);
        });
  }

  public Command runRollerCommand() {
    return run(() -> set(Speed.INTAKE));
  }

  public Command stopRollerCommand() {
    return run(() -> set(Speed.STOP));
  }

  /** checks if angle of pivot is within kPositionTolerance */
  public boolean isPositionWithinTolerance() {
    final Angle cur = Degrees.of(pivotEncoder.getPosition() * degreesPerRotation);
    final Angle target = setPointAngle;

    return cur.isNear(target, kPositionTolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rotations", pivotEncoder.getPosition());

    updatePivotPosition();
  }

  /** updates pivot position with slew */
  public void updatePivotPosition() {
    // update time
    final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
    final Time elapsedTime = currentTime.minus(lastUpdateTime);
    lastUpdateTime = currentTime;

    final double currentPosition = pivotEncoder.getPosition();
    final double targetPosition = setPointAngle.magnitude();

    // get new voltage according to pid controller
    double pidOutput = pivotMotorController.calculate(currentPosition,
        targetPosition);

    // get maximum allowed distance for this round
    final Voltage maxStep = Volts.of(kMaxPivotVoltageChange * elapsedTime.magnitude());

    // apply slew
    currentOutput = Volts.of(pidOutput > currentOutput.magnitude()
        ? Math.min(pidOutput, currentOutput.plus(maxStep).magnitude())
        : Math.max(pidOutput, currentOutput.minus(maxStep).magnitude()));

    // tuned pid for voltage
    pivotMotor.setVoltage(currentOutput);
  }

}
