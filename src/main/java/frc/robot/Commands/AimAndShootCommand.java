// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndShootCommand {
  public final PrepareShooterCommand prepareShooterCommand;

  /** Creates a new AimAndShootCommand. */
  public AimAndShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command run() {
    Commands.parallel(
        aimAndDriveCommand,
        Commands.waitSeconds(0.25)
            .andThen(prepareShotCommand),
        Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
            .andThen(feed()));
  }

}