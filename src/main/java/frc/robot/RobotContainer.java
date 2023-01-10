// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();



  public RobotContainer() {

    
    configureBindings();
  

  }

  
 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
      ()-> mDriver.getLeftY(), 
      ()-> mDriver.getLeftX(), 
      ()-> mDriver.getRightX()));

      mDriver.b().whileTrue(
        new StartEndCommand(
          () -> arm.run(.5),
          () -> arm.run(0),
          arm
        )
      );

      mOperator.a().toggleOnTrue(
        new StartEndCommand(
          () -> arm.run(0.5),
          () -> arm.run(0),
          arm
        )
      );
      
    }




    


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
