// currently hood only

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LandMarks;
import frc.robot.Commands.GeneralRobotCommands.HoodState;
import frc.robot.Commands.GeneralRobotCommands.ShooterState;
import frc.robot.Commands.PrepareShooterCommand.Shot;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareShooterCommand extends Command {

  public static class Shot {
    public final double shooterRPM;
    public final double hoodPosition;

    public Shot(double shooterRPM, double hoodPosition) {
      this.shooterRPM = shooterRPM;
      this.hoodPosition = hoodPosition;
    }
  }

  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
      // Reverse interpolates the value to find percentage distance between the two
      // known values
      (startValue, endValue, q) -> InverseInterpolator.forDouble()
          .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
      // uses previously generated value to interpolate shooter velocity and hood
      // position
      (startValue, endValue, t) -> new Shot(
          Interpolator.forDouble()
              .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
          Interpolator.forDouble()
              .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

  // to tune
  static {
    // distanceToShotMap.put(Meters.of(1.649778334547691), new Shot(5300, 0.187));
    // distanceToShotMap.put(Meters.of(0.5748198000485322), new Shot(5300, 0));
    // distanceToShotMap.put(Meters.of(2.4006216556413467), new Shot(5300, 0.304));
    // distanceToShotMap.put(Meters.of(1.8874167417587684), new Shot(5300, 0.226));
    // distanceToShotMap.put(Meters.of(1.378834779227017), new Shot(5300, 0.148));
    // distanceToShotMap.put(Meters.of(2.4854897119801356), new Shot(5300, 0.273));
    distanceToShotMap.put(Meters.of(4.021), new Shot(3950, 0.4));
    distanceToShotMap.put(Meters.of(3.460), new Shot(3800, 0.4));
    distanceToShotMap.put(Meters.of(2.845), new Shot(3700, 0.3));
    distanceToShotMap.put(Meters.of(2.500), new Shot(3500, 0.3));
    distanceToShotMap.put(Meters.of(2.216), new Shot(3400, 0.3));
    distanceToShotMap.put(Meters.of(1.841), new Shot(3300, 0.25));
    distanceToShotMap.put(Meters.of(1.456), new Shot(3200, 0.2));
    distanceToShotMap.put(Meters.of(1.223), new Shot(3000, 0.05));
    distanceToShotMap.put(Meters.of(4.266), new Shot(4000, 0.55));
    distanceToShotMap.put(Meters.of(5.123), new Shot(4200, 0.7));
  }

  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMapFeed = new InterpolatingTreeMap<>(
      // Reverse interpolates the value to find percentage distance between the two
      // known values
      (startValue, endValue, q) -> InverseInterpolator.forDouble()
          .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
      // uses previously generated value to interpolate shooter velocity and hood
      // position
      (startValue, endValue, t) -> new Shot(
          Interpolator.forDouble()
              .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
          Interpolator.forDouble()
              .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

  // to tune
  static {
    // distanceToShotMap.put(Meters.of(1.649778334547691), new Shot(5300, 0.187));
    // distanceToShotMap.put(Meters.of(0.5748198000485322), new Shot(5300, 0));
    // distanceToShotMap.put(Meters.of(2.4006216556413467), new Shot(5300, 0.304));
    // distanceToShotMap.put(Meters.of(1.8874167417587684), new Shot(5300, 0.226));
    // distanceToShotMap.put(Meters.of(1.378834779227017), new Shot(5300, 0.148));
    // distanceToShotMap.put(Meters.of(2.4854897119801356), new Shot(5300, 0.273));
    distanceToShotMapFeed.put(Meters.of(11), new Shot(5600, 0.804));
    distanceToShotMapFeed.put(Meters.of(8.544), new Shot(4300, 0.804));
    distanceToShotMapFeed.put(Meters.of(6.832), new Shot(3600, 0.804));
    distanceToShotMapFeed.put(Meters.of(6.480), new Shot(3000, 0.804));
  }

  private final ShooterSubsystem shooterSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final Distance distance;

  /** Creates a new AimShotCommand. */
  public PrepareShooterCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
      Distance distance) {
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.distance = distance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, hoodSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hoodSubsystem.setState(HoodState.AIMING);
    shooterSubsystem.setState(ShooterState.SHOOTING);

    // get appropriate rpm and hood position pair
    final Shot shot = distanceToShotMap.get(distance);

    // set subsystems with calculated values
    shooterSubsystem.set(RPM.of(shot.shooterRPM));
    hoodSubsystem.setPosition(shot.hoodPosition);
  }

  public void end(boolean interrupted) {
    hoodSubsystem.setState(HoodState.IDLE);
    shooterSubsystem.setState(ShooterState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
