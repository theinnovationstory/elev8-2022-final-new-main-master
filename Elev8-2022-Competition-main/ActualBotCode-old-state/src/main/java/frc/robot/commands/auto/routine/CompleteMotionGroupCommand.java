// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routine;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.drive.tester.DriveBySecondCoordinateCommand;
import frc.robot.commands.auto.drive.tester.DriveToACoordinateCommand;
import frc.robot.commands.auto.intake.AutonomousIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CompleteMotionGroupCommand extends SequentialCommandGroup {
  public double FIRST_X = 1.5;
  public double FIRST_Y = 0.0;
  public double ANGLE_SECOND = 154;

  /** Creates a new CompleteMotionGroupCommand. */
  public CompleteMotionGroupCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(
        new DriveToACoordinateCommand(driveSubsystem, FIRST_X, FIRST_Y),
        new AutonomousIntakeCommand(intakeSubsystem)));

    addCommands(new DriveBySecondCoordinateCommand(driveSubsystem, 154));
  }
}
