// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {

  WPI_TalonFX mTest;
  private XboxController mDriver = new XboxController(0);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();



  public RobotContainer() {

    mTest = new WPI_TalonFX(5, "rio");

    
    configureBindings();
  

  }

  
 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
      ()-> mDriver.getLeftY(), 
      ()-> mDriver.getLeftX(), 
      ()-> mDriver.getRightX()));
      
    }


    


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
