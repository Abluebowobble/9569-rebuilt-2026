package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.SilverKnightsLib.IsolatedSwerveController;
import frc.robot.LandMarks;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class Autons {
    private SwerveSubsystem swerve;
 
    public static Command testBlue1Auton(SwerveSubsystem swerveSubsystem, IsolatedSwerveController swerveController, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      swerveSubsystem.resetOdometry(new Pose2d(3.5, 5.79, Rotation2d.fromDegrees(0)));
    } else {
      swerveSubsystem.resetOdometry(new Pose2d(LandMarks.kFieldLength.in(Meter) - 3.5, LandMarks.kFieldWidth.in(Meter) - 5.79, Rotation2d.fromDegrees(180)));
    }

    return new SequentialCommandGroup(
        // drive over bump
        new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(5.74, 5.79, Rotation2d.fromDegrees(0), Feet.of(2))
        ),

        // intake fuel from neutral zone
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
              new DriveToPoseCommand(swerveController, swerveSubsystem,
                  new Pose2d(7.8, 5.79, Rotation2d.fromDegrees(-90), Feet.of(2))
              ),
              new DriveToPoseCommand(swerveController, swerveSubsystem,
                  new Pose2d(7.8, 4.14, Rotation2d.fromDegrees(-90))
              )
          ),
          intakeSubsystem.intakeCommand(),
          intakeSubsystem.runRollerCommand()
        ),

        // drive to bump area
        new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(5.79, 5.80, Rotation2d.fromDegrees(-180))
        ),

        // drive over bump
        new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(3.00, 5.80, Rotation2d.fromDegrees(-180))
        ),

        // positions itself to shoot
         new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(3.5, 3.97, Rotation2d.fromDegrees(180))
        ),

        shooterSubsystem.runCommand(RPM.of(5300)),

        // heads to depot to intake
        new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(1.52, 5.95, Rotation2d.fromDegrees(180))
        ),

        // intakes from depot
        new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(0.5, 5.95, Rotation2d.fromDegrees(180))
        ),

        // comes back from depot
        new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(1.52, 5.95, Rotation2d.fromDegrees(180))
        ),

        // positions itself to shoot
        new DriveToPoseCommand(swerveController, swerveSubsystem,
            new Pose2d(3.5, 3.97, Rotation2d.fromDegrees(180))
        ),

        // shoot
        shooterSubsystem.runCommand(RPM.of(5300))
    );
  }
}
