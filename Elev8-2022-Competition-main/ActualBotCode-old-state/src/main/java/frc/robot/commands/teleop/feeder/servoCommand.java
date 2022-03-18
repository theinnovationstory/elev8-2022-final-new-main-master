// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ServoFeederSubsystem;
import frc.robot.Constants.FeederConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class servoCommand extends CommandBase {
  /** Creates a new servoCommand. */

  ServoFeederSubsystem servoFeederSubsystem;

  public servoCommand(ServoFeederSubsystem servoFeederSubsystem) {
    this.servoFeederSubsystem = servoFeederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(servoFeederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.servoFeederSubsystem.setServoSpeed(FeederConstants.positionAngle);
    SmartDashboard.putBoolean("Servo up?", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.servoFeederSubsystem.setServoSpeed(FeederConstants.initialAngle);
    SmartDashboard.putBoolean("Servo up?", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !RobotContainer.joyD.getRawButton(4);
    return false;
  }
}
