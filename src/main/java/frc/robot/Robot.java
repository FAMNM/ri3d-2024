// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
  private final XboxController controller = new XboxController(0);

  @Override
  public void robotInit() {
    System.out.println("robotInit");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    motor.set(controller.getRightY());
  }
}
