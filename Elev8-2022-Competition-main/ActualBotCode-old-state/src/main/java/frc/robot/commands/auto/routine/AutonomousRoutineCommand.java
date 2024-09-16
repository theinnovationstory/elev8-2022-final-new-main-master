// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routine;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.intake.AutonomousIntakeCommand;
import frc.robot.commands.auto.shooter.AutonomousShooterCommand;
import frc.robot.commands.auto.intake.AutoStartServosCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.servo.ServoFeederSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRoutineCommand extends ParallelCommandGroup {
  /** Creates a new AutonomousRoutineCommand. */
  public AutonomousRoutineCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem,
      DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ServoFeederSubsystem initialServosSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoStartServosCommand(initialServosSubsystem));

    addCommands(new AutonomousShooterCommand(shooterSubsystem));

    addCommands(new AutonomousIntakeCommand(intakeSubsystem));

    addCommands(new CompleteMotionGroupCommand(feederSubsystem, driveSubsystem, intakeSubsystem));
  }
}
