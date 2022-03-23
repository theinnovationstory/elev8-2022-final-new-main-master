// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.servo.ServoFeederSubsystem;

public class AutoStartServosCommand extends CommandBase {
  private ServoFeederSubsystem initialServosSubsystem;
  private double FIRST_ANGLE = 100;
  private double SECOND_ANGLE = -100;
  /** Creates a new Auto_intake_Commands. */
  public AutoStartServosCommand(ServoFeederSubsystem initialServosSubsystem) {
    this.initialServosSubsystem = initialServosSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.initialServosSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initialServosSubsystem.setServo1Speed(FIRST_ANGLE);
    this.initialServosSubsystem.setServo2Speed(SECOND_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
