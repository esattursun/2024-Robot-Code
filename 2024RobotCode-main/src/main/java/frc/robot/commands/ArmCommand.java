// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private final Supplier<Double> controller;

  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem armSubsystem, Supplier<Double> controller) {
    this.armSubsystem = armSubsystem;
    this.controller = controller;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllers = controller.get();
    armSubsystem.manuelArmControl(controllers);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.manuelArmControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
}
