// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Commands.GeneralRobotCommands.ShooterState;
import frc.robot.Commands.GeneralRobotCommands.SwerveState;
import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
  // motors
  private final SparkMax leftShooterMotor = new SparkMax(HardwareMap.SHOOTER_LEFT, MotorType.kBrushless);
  private final SparkMax middleShooterMotor = new SparkMax(HardwareMap.SHOOTER_MIDDLE, MotorType.kBrushless);
  private final SparkMax rightShooterMotor = new SparkMax(HardwareMap.SHOOTER_RIGHT, MotorType.kBrushless);

  // encoders
  private final RelativeEncoder leftEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder middleEncoder = middleShooterMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightShooterMotor.getEncoder();

  // disablers
  public boolean isDisabledLeft = false;
  public boolean isDisabledMiddle = false;
  public boolean isDisabledRight = false;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(500);

  private final SparkClosedLoopController l_controller = leftShooterMotor.getClosedLoopController();
  private final SparkClosedLoopController m_controller = middleShooterMotor.getClosedLoopController();
  private final SparkClosedLoopController r_controller = rightShooterMotor.getClosedLoopController();

  // pidf
  private AngularVelocity targetRPM = RPM.of(0); // desired RPM we want the wheels to turn at

  private static final AngularVelocity kVelocityTolerance = RPM.of(300); // if current RPM is within desired RPM +-

  // velocity tolerance,
  // then its within tolerance
  private Voltage voltage = Volts.of(0);

  public static final AngularVelocity kStartingVelocity = RPM.of(2000);

  private ShooterState shooterState = ShooterState.IDLE;

  private static final AngularVelocity kVelocityToleranceForShooting = RPM.of(2000);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // initialize motors
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .voltageCompensation(12.0)
        .openLoopRampRate(1)
        .closedLoopRampRate(1)
        .inverted(true);
    leftConfig.closedLoop
        // .pid(0.0001, 0, 0.001, ClosedLoopSlot.kSlot0).feedForward // test
        // .sv(0.115, 0.00203, ClosedLoopSlot.kSlot0); // might wanna increase kV
        .pid(0.00001, 0, 0.001, ClosedLoopSlot.kSlot0).feedForward // test
        .sv(0.115, 0.00203, ClosedLoopSlot.kSlot0); // might wanna increase kV
    leftConfig.smartCurrentLimit(70, 70);

    // double check this
    leftConfig.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);

    leftShooterMotor.configure(
        leftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig middleConfig = new SparkMaxConfig();
    middleConfig
        .voltageCompensation(12.0)
        .openLoopRampRate(1)
        .closedLoopRampRate(1)
        .inverted(false);
    middleConfig.closedLoop
        .pid(0.0001, 0, 0.001, ClosedLoopSlot.kSlot0).feedForward // test
        .sv(0.115, 0.00203, ClosedLoopSlot.kSlot0); // might wanna increase kV
    middleConfig.smartCurrentLimit(70, 70);

    // double check this
    middleConfig.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);

    middleShooterMotor.configure(
        middleConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .voltageCompensation(12.0)
        .openLoopRampRate(1)
        .closedLoopRampRate(1)
        .inverted(false);
    rightConfig.closedLoop
        .pid(0.0001, 0, 0.001, ClosedLoopSlot.kSlot0).feedForward // test
        .sv(0.115, 0.00203, ClosedLoopSlot.kSlot0); // might wanna increase kV
    rightConfig.smartCurrentLimit(70, 70);

    // double check this
    rightConfig.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);

    rightShooterMotor.configure(
        rightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    setDefaultCommand(idle());

    SmartDashboard.putBoolean("Left Shooter Enabled", true);
    SmartDashboard.putBoolean("Middle Shooter Enabled", true);
    SmartDashboard.putBoolean("Right Shooter Enabled", true);
  }

  /** sets voltage of all motors given Voltage enum */
  public void setVoltage(Voltage voltage) {
    leftShooterMotor.setVoltage(voltage.magnitude());
    middleShooterMotor.setVoltage(voltage.magnitude());
    rightShooterMotor.setVoltage(voltage.magnitude());
  }

  /**
   * this function updates the rpm based on the value in smart dashboard, updates
   * each motor separately
   */
  public void updateSpeedWithSmartDashboard() {
    double targetL = SmartDashboard.getNumber("Left Pivot RPM", 0);
    double targetM = SmartDashboard.getNumber("Middle Pivot RPM", 0);
    double targetR = SmartDashboard.getNumber("Right Pivot RPM", 0);
    double[] targets = { targetL, targetM, targetR };

    for (int i = 0; i < 3; i++) {
      set(Volts.of(targets[i]));
    }
  }

  /** set target RPM for all motors */
  public void set(DoubleSupplier rpm) {
    set(RPM.of(rpm.getAsDouble()));
  }

  public void set(AngularVelocity rpm) {
    targetRPM = RPM.of(MathUtil.clamp(rpm.magnitude(), 0, 5600));
    // l_controller.setSetpoint(rpm.magnitude(), ControlType.kVelocity,
    // ClosedLoopSlot.kSlot0);
    // m_controller.setSetpoint(rpm.magnitude(), ControlType.kVelocity,
    // ClosedLoopSlot.kSlot0);
    // r_controller.setSetpoint(rpm.magnitude(), ControlType.kVelocity,
    // ClosedLoopSlot.kSlot0);

    if (!SmartDashboard.getBoolean("Left Shooter Enabled", true)) {
      l_controller.setSetpoint(0, ControlType.kVelocity);
    } else {
      l_controller.setSetpoint(rpm.magnitude(), ControlType.kVelocity,
          ClosedLoopSlot.kSlot0);
    }

    if (!SmartDashboard.getBoolean("Middle Shooter Enabled", true)) {
      m_controller.setSetpoint(0, ControlType.kVelocity);
    } else {
      m_controller.setSetpoint(rpm.magnitude(), ControlType.kVelocity,
          ClosedLoopSlot.kSlot0);
    }

    if (!SmartDashboard.getBoolean("Right Shooter Enabled", true)) {
      r_controller.setSetpoint(0, ControlType.kVelocity);
    } else {
      r_controller.setSetpoint(rpm.magnitude(), ControlType.kVelocity,
          ClosedLoopSlot.kSlot0);
    }
  }

  public boolean shouldFeed() {
    double currentMinRpm = getMinimumVelocity();
    return MathUtil.isNear(targetRPM.magnitude(), currentMinRpm, kVelocityToleranceForShooting.magnitude());
  }

  public double getMinimumVelocity() {
    double currentMinRpm = Math.min(leftEncoder.getVelocity(),
        Math.min(middleEncoder.getVelocity(), rightEncoder.getVelocity()));
    return currentMinRpm;
  }

  public void set(Voltage volts) {
    leftShooterMotor.setVoltage(volts.magnitude());
    middleShooterMotor.setVoltage(volts.magnitude());
    rightShooterMotor.setVoltage(volts.magnitude());
  }

  /** checks if each shooter has their velocity is within tolerance */
  public boolean isVelocityWithinTolerance() {
    return targetRPM.isNear(RPM.of(leftEncoder.getVelocity()), kVelocityTolerance);
  }

  /** sets voltage to shoot in front of Hub */
  public Command runCommand(AngularVelocity rpm) {
    return run(() -> targetRPM = rpm).alongWith(Commands.runOnce(() -> setState(ShooterState.SHOOTING)));
  }

  /** sets voltage to shoot in front of Hub */
  public Command runTestCommand(DoubleSupplier rpm) {
    return run(() -> targetRPM = RPM.of(rpm.getAsDouble()));
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> set(voltage));
  }

  @Override
  public Command idle() {
    return runOnce(() -> {
      targetRPM = RPM.of(kStartingVelocity.magnitude());
    }).alongWith(Commands.runOnce(() -> setState(ShooterState.IDLE)));
  }

  public void setState(ShooterState shooterState) {
    this.shooterState = shooterState;
  }

  public ShooterState getState() {
    return shooterState;
  }

  @Override
  public void periodic() {
    targetRPM = RPM.of(MathUtil.clamp(targetRPM.magnitude(), 0, 5600));

    // double averageLeftMotorOutput = currentFilter.calculate(leftMotorOutput);
    // double averageMiddleMotorOutput = currentFilter.calculate(middleMotorOutput);
    // double averageRightMotorOutput = currentFilter.calculate(rightMotorOutput);

    if (!SmartDashboard.getBoolean("Left Shooter Enabled", true)) { // MathUtil.isNear(65, averageLeftMotorOutput, 5)
      l_controller.setSetpoint(0, ControlType.kVelocity);
    } else {
      l_controller.setSetpoint(targetRPM.magnitude(), ControlType.kVelocity);
    }

    if (!SmartDashboard.getBoolean("Middle Shooter Enabled", true)) { // MathUtil.isNear(65, averageMiddleMotorOutput,
      m_controller.setSetpoint(0, ControlType.kVelocity);
    } else {
      m_controller.setSetpoint(targetRPM.magnitude(), ControlType.kVelocity);
    }
    if (!SmartDashboard.getBoolean("Right Shooter Enabled", true)) { // MathUtil.isNear(65, averageMiddleMotorOutput, 5)
      r_controller.setSetpoint(0, ControlType.kVelocity);
    } else {
      r_controller.setSetpoint(targetRPM.magnitude(), ControlType.kVelocity);
    }

    // uncomment below to tune
    // updateSpeedWithSmartDashboard();

    SmartDashboard.putBoolean("is velocity of shooters within tolerance", isVelocityWithinTolerance());

    // telemetry
    SmartDashboard.putData(this);
  }

  public double progress() {
    if (isVelocityWithinTolerance()) {
      return 1.0;
    }

    AngularVelocity avgVelocity = RPM
        .of(Math.abs((leftEncoder.getVelocity() + middleEncoder.getVelocity() + rightEncoder.getVelocity()) / 3.0));

    double targetRpmMagnitude = targetRPM.magnitude();
    // dont divide by zero :)
    if (targetRpmMagnitude < 1e-9) {
      return 1.0;
    }

    double error = Math.abs(avgVelocity.magnitude() - targetRpmMagnitude);
    return MathUtil.clamp(1.0 - (error / targetRpmMagnitude), 0.0, 1.0);
  }

  @Override
  public String toString() {
    if (shooterState == ShooterState.SHOOTING) {
      if (!isVelocityWithinTolerance())
        return "NOT READY";
      else
        return "READY";
    } else {
      return "IDLE";
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("left RPM", () -> leftEncoder.getVelocity(), null);
    builder.addDoubleProperty("middle RPM", () -> middleEncoder.getVelocity(), null);
    builder.addDoubleProperty("right RPM", () -> rightEncoder.getVelocity(), null);
    // addd current property later
    builder.addBooleanProperty("is Velocity within tolerance", this::isVelocityWithinTolerance, null);
    builder.addDoubleProperty("voltage (each)", () -> voltage.magnitude(), null);
    builder.addDoubleProperty("Target rpm", () -> targetRPM.magnitude(), null);

    builder.addDoubleProperty("Right Shooter Current Supply (A)", () -> rightShooterMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Right Shooter Temperature", () -> rightShooterMotor.getMotorTemperature(), null);
    builder.addDoubleProperty("Middle Shooter Temperature", () -> middleShooterMotor.getMotorTemperature(), null);
    builder.addDoubleProperty("Middle Shooter Current Supply (A)", () -> middleShooterMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Left Shooter Temperature", () -> leftShooterMotor.getMotorTemperature(), null);
    builder.addDoubleProperty("Left Shooter Current Supply (A)", () -> leftShooterMotor.getOutputCurrent(), null);

    builder.addBooleanProperty("left disabled?", () -> isDisabledLeft, null);
    builder.addBooleanProperty("middle disabled?", () -> isDisabledMiddle, null);
    builder.addBooleanProperty("right disabled?", () -> isDisabledRight, null);

    builder.addStringProperty("Current Shooter State", this::toString, null);
  }
}