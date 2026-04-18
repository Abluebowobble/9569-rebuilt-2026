// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;



import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Commands.GeneralRobotCommands.IntakeState;
import frc.robot.Commands.GeneralRobotCommands.RollerState;
import frc.SilverKnightsLib.OPRSlewRateLimiter;
import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class IntakeSubsystem extends SubsystemBase {

  // electronics
  private final SparkMax pivotMotor = new SparkMax(HardwareMap.INTAKE_PIVOT, MotorType.kBrushless);
  private final SparkMax rollerMotor = new SparkMax(HardwareMap.INTAKE_ROLLER, MotorType.kBrushless);
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(HardwareMap.INTAKE_ENCODER);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  // set point for pid
  private double pivotSetPoint = 0;

  // range of allowed positions
  private static final double kPositionTolerance = 0.004; // to tune

  // gear reduction
  private final Angle kDegreesPerRotation = Degrees.of(2);

  // private final SparkClosedLoopController controller =
  // pivotMotor.getClosedLoopController();
  private final PIDController controller = new PIDController(6, 0, 0);
  private IntakeState intakeState = IntakeState.STOWED;
  private RollerState rollerState = RollerState.STOP;

  // speed for roller motor
  public enum Speed { // 0.08969272724231818
    STOP(0),
    INTAKE(0.95), // to tune
    REVERSE(-0.9); // to tune

    private final double percentOutput;

    private Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }

    public Voltage voltage() {
      return Volts.of(percentOutput * 12.0);
    }
  }

  double zero = 0.78;

  // set angle for pivot motor
  public enum Position {
    STOWED(0.0), // to update 0.7783878944596974
    AGITATE(-0.745), // to update 0.7527 ,
    INTAKE(-0.683); // to update 0.08388367709709192 // 0.0838 

    private final double percentRotation;

    private Position(double percentRotation) {
      this.percentRotation = percentRotation;
    }

    public double percentRotation() {
      return percentRotation;
    }
  }

  OPRSlewRateLimiter slewRateLimit = new OPRSlewRateLimiter(50, 100);

  public IntakeSubsystem() {
    SparkBaseConfig pivotConfig = new SparkMaxConfig();
    // pivotConfig.closedLoop.p(0.7, ClosedLoopSlot.kSlot0); // 0.7
    pivotConfig.smartCurrentLimit(40, 20);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkBaseConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.smartCurrentLimit(120, 60);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller.enableContinuousInput(0, 1);
    controller.calculate(Double.MAX_VALUE, 0);

    // pivotEncoder.setPosition(0);
    // SmartDashboard.putData("IntakeDown", intakeCommand());
    setDefaultCommand(idle());
  }

  /** set roller to percentage output given speed enum */
  public void set(Speed speed) {
    rollerMotor.setVoltage(speed.voltage());
  }

  /** set pivot motor to position given Position enum */
  public void set(Position position) {
    pivotSetPoint = position.percentRotation();
    // controller.setSetpoint(position.degrees().div(kDegreesPerRotation).magnitude(),
    // ControlType.kPosition,
    // ClosedLoopSlot.kSlot0);
  }

  /** set pivot motor to go to intake position */
  public Command intakePositionCommand() {
    return runOnce(() -> set(Position.INTAKE)).alongWith(Commands.run(() -> setState(IntakeState.INTAKE)));
  }

  /** Return to home position */
  public Command returnPositionCommand() {
    return runOnce(() -> set(Position.STOWED)).alongWith(Commands.run(() -> setState(IntakeState.STOWED)));
  }

  public void sinusoidalPivot(Timer timer) {
    final double amplitude = 0.05;
    final double center = Position.AGITATE.percentRotation();
    final double frequency = 1.25;
    final double omega = 2 * Math.PI * frequency;

    double time = timer.get();

    double rotations = center + amplitude * Math.sin(omega * time);

    pivotSetPoint = rotations;

    // controller.setSetpoint(
    // rotations,
    // ControlType.kPosition,
    // ClosedLoopSlot.kSlot0);
  }

  public Command sinusoidalPivotCommand() {
    Timer timer = new Timer();

    return runOnce(() -> set(Position.AGITATE))
        .andThen(run(() -> {
          sinusoidalPivot(timer);
        })).beforeStarting(timer::restart);
  }

  /**
   * humps balls in basket, goes back to intake position and stops on interrupt
   */
  public Command agitatePivotCommand(IntakeState prevState) {
    return runOnce(() -> set(Speed.INTAKE))
        .andThen(Commands.runOnce(() -> setState(IntakeState.AGITATING)))
        .andThen(
            sinusoidalPivotCommand())
        .handleInterrupt(() -> {
          if (prevState == IntakeState.STOWED) {
            set(Position.STOWED);
          } else if (prevState == IntakeState.INTAKE) {
            set(Position.INTAKE);
          }

          set(Speed.STOP);
          setState(prevState);
        });
    // .onlyIf(() -> !isStowed());
  }

  private boolean agitate = false;

  public Command agitateWithOverrideCommand(BooleanSupplier reverseButton) {
    Timer timer = new Timer();
    return run(() -> {
      if (reverseButton.getAsBoolean()) {
        set(Position.INTAKE);
        agitate = false;
        return;
      } else if (!agitate) {
        timer.restart();
        agitate = true;
      }

      if (agitate) {
        sinusoidalPivot(timer);
        set(Speed.INTAKE);
      }
      setState(IntakeState.AGITATING);
    }).beforeStarting(() -> {
      timer.restart();
      set(Position.INTAKE);
      set(Speed.INTAKE);
      setState(IntakeState.AGITATING);
    }).handleInterrupt(() -> {
      set(Position.INTAKE);
      set(Speed.STOP);
      setState(IntakeState.INTAKE);
    });
  }

  public boolean isStowed() {
    return pivotSetPoint == Position.STOWED.percentRotation();
  }

  public Command runRollerCommand() {
    return run(() -> set(Speed.INTAKE)).alongWith(Commands.run(() -> setState(RollerState.RUNNING)));
  }

  public Command stopRollerCommand() {
    return run(() -> set(Speed.STOP)).alongWith(Commands.run(() -> setState(RollerState.STOP)));
  }

  public Command reverseRollerCommand() {
    return run(() -> set(Speed.REVERSE)).alongWith(Commands.run(() -> setState(RollerState.REVERSE)));
  }

  @Override
  public Command idle() {
    return runOnce(() -> {
      set(Speed.STOP);
    }).alongWith(Commands.run(() -> setState(RollerState.STOP)));
  }

  public void setState(IntakeState intakeState) {
    this.intakeState = intakeState;
  }

  public Command intakeCommand() {
    return run(() -> {
      setState(IntakeState.INTAKE);
      set(Position.INTAKE);
      set(Speed.INTAKE);
    });
  }

  public Command togglePositionCommand() {
    return defer(() -> {
      if (intakeState == IntakeState.STOWED) {
        intakeState = IntakeState.INTAKE;
        return runOnce(() -> set(Position.INTAKE));
      } else if (intakeState == IntakeState.INTAKE) {
        intakeState = IntakeState.STOWED;
        return runOnce(() -> set(Position.STOWED));
      }

      return idle();
    });
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }

  public void setState(RollerState rollerState) {
    this.rollerState = rollerState;
  }

  public RollerState getRollerState() {
    return rollerState;
  }

  /** checks if angle of pivot is within kPositionTolerance */
  public boolean isPositionWithinTolerance() {
    return MathUtil.isNear(absEncoder.get(), pivotSetPoint, kPositionTolerance);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putData(this);
    // SmartDashboard.putNumber("setpoint", pivotSetPoint);
    updatePivotPositionCommand();
  }

  public void updatePivotPositionCommand() {
    double output = controller.calculate(absEncoder.get() - zero, pivotSetPoint);
    pivotMotor.set(output);
  }

  @Override
  public String toString() {
    switch (rollerState) {
      case RUNNING:
        return "INTAKING";
      case REVERSE:
        return "REVERSING";
      case STOP:
      default:
        return "STOPPED";
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position (rotations)", () -> pivotEncoder.getPosition(), null);
    builder.addDoubleProperty("Roller Velocity (RPM)", () -> rollerMotor.getEncoder().getVelocity(), null);
    builder.addDoubleProperty("Roller Supply Current (A)", () -> rollerMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Pivot Supply Current (A)", () -> pivotMotor.getOutputCurrent(), null);
    builder.addStringProperty("Current Intake Roller State", this::toString, null);
    builder.addDoubleProperty("Absolute Encoder Position", () -> absEncoder.get() - zero, null);
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
