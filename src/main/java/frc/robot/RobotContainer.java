// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.custom.Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

  WPI_TalonFX mTest;
  private Controller m_driverController = new Controller(1);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    mTest = new WPI_TalonFX(5, "rio");

    // Configure the trigger bindings
    configureBindings();
  

  }

  
 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
      ()-> m_driverController.getLeftY() * 0.2, 
      ()-> m_driverController.getLeftX() * 0.2, 
      ()-> m_driverController.getRightX() * 0.2));


      // m_driverController.a().whenHeld(
      //   new InstantCommand(
      //     () -> arm.run(-0.2)
      //   )
      //   ).whenReleased(
      //     () -> arm.run(0)
      //   );
     

    }


    


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
