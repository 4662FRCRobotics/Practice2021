// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private CANSparkMax m_leftController1;
  private CANSparkMax m_leftController2;
  private CANSparkMax m_rightController1;
  private CANSparkMax m_rightController2;
  private SpeedControllerGroup m_leftControllerGroup;
  private SpeedControllerGroup m_rightControllerGroup;
  private DifferentialDrive m_differentialdrive;
  
  public DriveSubsystem() {
    
    m_leftController1 = new CANSparkMax(2,MotorType.kBrushless);
    m_leftController2 = new CANSparkMax(3,MotorType.kBrushless);
    m_rightController1 = new CANSparkMax(4,MotorType.kBrushless);
    m_rightController2 = new CANSparkMax(5,MotorType.kBrushless);
    
    m_rightController1.restoreFactoryDefaults(); 
    m_rightController2.restoreFactoryDefaults();
    m_leftController1.restoreFactoryDefaults();
    m_leftController2.restoreFactoryDefaults();

    /*
    m_leftControllerGroup= new SpeedControllerGroup(m_leftController1, m_leftController2);
    m_rightControllerGroup= new SpeedControllerGroup(m_rightController1, m_rightController2);
    m_leftControllerGroup.setInverted(false);
    m_rightControllerGroup.setInverted(false);
     m_differentialdrive= new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);
    */
    
    m_leftController2.follow(m_leftController1);
    m_rightController2.follow(m_rightController1);
    m_differentialdrive= new DifferentialDrive(m_leftController1, m_rightController1);
    m_rightController1.setInverted(DriveConstants.kIS_DRIVE_INVERTED);
    m_leftController1.setInverted(DriveConstants.kIS_DRIVE_INVERTED);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void arcadedrive(final double velocity,final double heading) {
  m_differentialdrive.arcadeDrive(velocity,-1*heading);
}


}
