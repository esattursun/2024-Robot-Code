// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ControlCommand extends Command {
  /** Creates a new ControlCommand. */
  int ShooterMotor_1;
  int ShooterMotor_2;

  int IntakeMotor_1;
  int IntakeMotor_2;

  int ArmMotor_1;
  int ArmMotor_2;

  int ArmLockMotor_1;
  
  public ControlCommand(int ShooterMotor_1, int ShooterMotor_2, int IntakeMotor_1, int IntakeMotor_2, int ArmMotor_1, int ArmMotor_2,int ArmLockMotor_1) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ShooterMotor_1 = ShooterMotor_1;
    this.ShooterMotor_2 = ShooterMotor_2;

    this.IntakeMotor_1 = IntakeMotor_1;
    this.IntakeMotor_2 = IntakeMotor_2;

    this.ArmMotor_1 = ArmMotor_1;
    this.ArmMotor_2 = ArmMotor_2;

    this.ArmLockMotor_1 = ArmLockMotor_1;

    addRequirements(RobotContainer.shooterSubsystem,RobotContainer.armSubsystem,RobotContainer.intakeSubsystem,RobotContainer.armlocksubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  RobotContainer.shooterSubsystem.setMotor1(ShooterMotor_1);
  RobotContainer.shooterSubsystem.setMotor2(ShooterMotor_2);

  RobotContainer.intakeSubsystem.setMotor1(IntakeMotor_1);
  RobotContainer.intakeSubsystem .setMotor2(IntakeMotor_2); 

  RobotContainer.armSubsystem.setmotor1(ArmMotor_1);
  RobotContainer.armSubsystem.setmotor2(ArmMotor_2);
  
  RobotContainer.armlocksubsystem.setmotor1(ArmLockMotor_1);
  
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
