// package frc.robot.Commands;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Subsystems.ConveyorSubsystem;
// import frc.robot.Subsystems.FeederSubsystem;
// import frc.robot.Subsystems.HoodSubsystem;
// import frc.robot.Subsystems.IntakeSubsystem;
// import frc.robot.Subsystems.ShooterSubsystem;
// import frc.robot.Subsystems.SwerveSubsystem;
// import frc.robot.Subsystems.Vision;

// public class GeneralRobotCommands {

//     Vision vision;
//     SwerveSubsystem swerveSubsystem;
//     ShooterSubsystem shooterSubsystem;
//     IntakeSubsystem intakeSubsystem;
//     HoodSubsystem hoodSubsystem;
//     FeederSubsystem feederSubsystem;
//     ConveyorSubsystem conveyorSubsystem;
//     DoubleSupplier leftSupplier;
//     DoubleSupplier rightSupplier;

//     public GeneralRobotCommands(Vision vision, SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem,
//             IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem,
//             ConveyorSubsystem conveyorSubsystem, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {

//     }

//     public Command AimAndShootCommand() {
//         AimCommand aimCommand = new AimCommand(swerveSubsystem, leftSupplier, rightSupplier);
//         PrepareShooterCommand prepareShooterCommand = new PrepareShooterCommand(shooterSubsystem, hoodSubsystem,
//                 () -> swerveSubsystem.getPose());

//         return Commands.deadline(
//                 Commands.waitUntil(() -> aimCommand.isAimed()
//                         && prepareShooterCommand.isReadyToShoot())
//                         .andThen(feed()),

//                 aimCommand,
//                 Commands.waitSeconds(0.25).andThen(prepareShooterCommand));
//     }

//     public Command feed() {
        
//     }
// }
