// package frc.robot.Commands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.SwerveSubsystem;

// public class Autons {
//     private SwerveSubsystem swerve;

//     public Autons(SwerveSubsystem swerve) {
//         this.swerve = swerve;
//     }

//     public Command testDriveForwardAutonFieldRelative() {
//         return swerve.driveToPose(new Pose2d(Meter.of(2.47), Meter.of(4), new Rotation2d(0)));
//     }

//     public Command testDriveForwardAutonRobotRelative() {
//         return swerve.driveToPose(new Pose2d(Meter.of(0.5), Meter.of(0), new Rotation2d(0)));
//     }

//     public Command simpleAuton() {
        
//     }

//     public Command testSpinAuton() {
//         return swerve.driveToPose(new Pose2d(Meter.of(0), Meter.of(0), new Rotation2d(Degree.of(90))));
//     }
// }
