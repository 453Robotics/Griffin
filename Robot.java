/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private static final int leftDeviceID = 8; 
  private static final int rightDeviceID = 3;
  private static final int leftfDeviceID = 2; 
  private static final int rightfDeviceID = 4;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftfMotor;
  private CANSparkMax m_rightfMotor;
  @Override
  public void robotInit() {
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    m_leftfMotor = new CANSparkMax(leftfDeviceID, MotorType.kBrushless);
    m_rightfMotor = new CANSparkMax(rightfDeviceID, MotorType.kBrushless);

    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftfMotor.restoreFactoryDefaults();
    m_rightfMotor.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_leftfMotor.follow(m_leftMotor);
    m_rightfMotor.follow(m_rightMotor);

    m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);
  
  }

  @Override
  public void teleopPeriodic() {
    //this this broken. IT ONLY GOES BACK AND FORTH
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_leftStick.getY());
  }
}
