package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flipper;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.controls.deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final Flipper flipper = new Flipper();
  private final Claw claw = new Claw();



  public RobotContainer() {

    
    configureBindings();
  

  }

 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
        ()-> deadbander.applyLinearScaledDeadband(mDriver.getLeftY(), 0.2), 
        ()-> deadbander.applyLinearScaledDeadband(mDriver.getLeftX(), 0.2), 
        ()-> deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.2)));

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
      
      mOperator.b().whileTrue(
        new StartEndCommand(
          () -> flipper.run(0.25),
          flipper::stop,
          flipper
        )
      );

      mOperator.b().whileTrue(
        new StartEndCommand(
          () -> flipper.run(-0.25),
          flipper::stop,
          flipper
        )
      );

    }

      

    


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  }
