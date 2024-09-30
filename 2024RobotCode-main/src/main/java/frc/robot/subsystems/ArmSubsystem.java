// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax armLmotor;
  private CANSparkMax armRmotor;
  private DutyCycleEncoder m_angleEncoder;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armLmotor = new CANSparkMax(Constants.UpSystemConstants.armLmotorId, MotorType.kBrushless);
    armRmotor = new CANSparkMax(Constants.UpSystemConstants.armRmotorId, MotorType.kBrushless);

    armRmotor.setInverted(true);

    armLmotor.setIdleMode(IdleMode.kBrake);
    armRmotor.setIdleMode(IdleMode.kBrake);

    //m_angleEncoder = armLmotor.get
    m_angleEncoder = new DutyCycleEncoder(0);
    m_angleEncoder.reset();
  }

  public void manuelArmControl(double controller){
    armLmotor.set(controller * .5);
    armRmotor.set(controller * .5);
  }

  public double getEncoderDegrees(){
    return ((m_angleEncoder.getAbsolutePosition() * 360) - 39);
  }

  public void setmotor1(double speed){
    armRmotor.set(speed);
  }
  public void setmotor2(double speed){
    armLmotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Degrees: ", getEncoderDegrees());
  }
}
