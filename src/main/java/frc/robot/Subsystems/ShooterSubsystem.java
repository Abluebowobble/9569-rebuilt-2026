// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private final SparkMax[] motors = { leftShooterMotor, middleShooterMotor, rightShooterMotor };

  // encoders
  private final RelativeEncoder leftEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder middleEncoder = middleShooterMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightShooterMotor.getEncoder();
  private final RelativeEncoder[] encoders = { leftEncoder, middleEncoder, rightEncoder };

  // pidf
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0, 0);//0.15, 0, 0.03); 
  private final PIDController controller = new PIDController(0.09, 0, 0); // to tune
  private double targetRPM = 0; // desired RPM we want the wheels to turn at

  private static final double kVelocityTolerance = 1; // if current RPM is within desired RPM +- velocity tolerance,
                                                      // then its within tolerance

  private Voltage voltage = Volts.of(0);

  // speed for roller motor
  public enum Speed {
    STOP(Volts.of(0)),
    INFRONTOFHUB(Volts.of(10)); // to tune

    private final Voltage voltage;

    private Speed(Voltage voltage) {
      this.voltage = voltage;
    }

    public Voltage voltage() {
      return voltage;
    }
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // initialize motors
    SparkBaseConfig config = new SparkMaxConfig();
    leftShooterMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    middleShooterMotor.configure(config.inverted(true), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightShooterMotor.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Set Left Pivot RPM", 0);
    SmartDashboard.putNumber("Set Middle Pivot RPM", 0);
    SmartDashboard.putNumber("Set Right Pivot RPM", 0);
  }

  /** sets voltage of all motors given Voltage enum */
  public void setVoltage(Voltage voltage) {
    leftShooterMotor.setVoltage(voltage.magnitude());
    middleShooterMotor.setVoltage(voltage.magnitude());
    rightShooterMotor.setVoltage(voltage.magnitude());
  }

  /**
   * updates speed of given motor via pidf, requires RelativeEncoder encoder,
   * SparkMax motor
   */
  public void updateCurrentSpeedOfMotor(RelativeEncoder encoder, SparkMax motor) {
    double pidVoltage = controller.calculate(encoder.getVelocity(), targetRPM);
    motor.setVoltage(feedForward.calculate(targetRPM) + pidVoltage);
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

  /** update speed for all three motors */
  public void updateCurrentSpeed() {
    // updateCurrentSpeedOfMotor(encoders[0], motors[0]);
    for (int i = 0; i < 3; i++) {
      updateCurrentSpeedOfMotor(encoders[i], motors[i]);
    }
  }

  /** set target RPM for all motors */
  public void set(double rpm) {
    targetRPM = rpm;
  }

  public void set(Speed volts) {
    voltage = volts.voltage();
    leftShooterMotor.setVoltage(volts.voltage());
    middleShooterMotor.setVoltage(volts.voltage());
    rightShooterMotor.setVoltage(volts.voltage());
  }

  public void set(Voltage volts) {
    leftShooterMotor.setVoltage(volts.magnitude());
    middleShooterMotor.setVoltage(volts.magnitude());
    rightShooterMotor.setVoltage(volts.magnitude());
  }

  /** sets rpm to 0 */
  public void stop() {
    set(0);
  }

  /** checks if each shooter has their velocity is within tolerance */
  public boolean isVelocityWithinTolerance() {
    for (RelativeEncoder e : encoders) {
      double currentRPM = e.getVelocity();
      if (!MathUtil.isNear(targetRPM, currentRPM, kVelocityTolerance)) {
        return false;
      }
    }

    return true;
  }

  /** sets voltage to shoot in front of Hub */
  public Command runCommand() {
    // return runOnce(() -> set(rpm))
    // .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    return startEnd(() -> set(Speed.INFRONTOFHUB), () -> set(Speed.STOP));
  }

  @Override
  public void periodic() {
    // pid
    // updateCurrentSpeed();

    // uncomment below to tune
    // updateSpeedWithSmartDashboard();

    // telemetry
    SmartDashboard.putNumber("left RPM", leftEncoder.getVelocity());
    SmartDashboard.putNumber("middle RPM", middleEncoder.getVelocity());
    SmartDashboard.putNumber("right RPM", rightEncoder.getVelocity());

    SmartDashboard.putNumber("shooter voltage (each)", voltage.magnitude());
  }
}