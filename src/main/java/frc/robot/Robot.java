// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
  private final CANSparkMax motor = new CANSparkMax(2, MotorType.kBrushless);
  private final XboxController controller = new XboxController(0);
  // private final SparkPIDController pid_controller = motor.getPIDController();
  private final RelativeEncoder encoder = motor.getEncoder();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  // public double rotations = 0.0;
  public boolean claw_closed = false;

  public int status = 0;

  @Override
  public void robotInit() {
    System.out.println("robotInit");

    // PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 4;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    // pid_controller.setP(kP);
    // pid_controller.setI(kI);
    // pid_controller.setD(kD);
    // pid_controller.setIZone(kIz);
    // pid_controller.setFF(kFF);
    // pid_controller.setOutputRange(kMinOutput, kMaxOutput);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (controller.getAButtonPressed()) {
      claw_closed = !claw_closed;
    }

    if (claw_closed) {
      if (encoder.getPosition() > 0.025) {

        if (status != 1) {
          System.out.println("Retracting...");
          status = 1;
        }

        if (encoder.getPosition() > 0.2) {
          motor.set(-0.1);
        } else {
          motor.set(-0.025);
        }
      } else {

        if (status != 0) {
          System.out.println("Done");
          status = 0;
        }

        motor.set(0);
      }
    } else {
      if (encoder.getPosition() < 0.2) {

        if (status != 2) {
          System.out.println("Grabbing...");
          status = 2;
        }

        motor.set(0.025);
      } else {

        if (status != 0) {
          System.out.println("Done");
          status = 0;
        }

        motor.set(0);
      }
    }

    // pid_controller.setReference(claw_closed ? 0.2 : 0.05,
    // CANSparkMax.ControlType.kPosition);

  }
}
