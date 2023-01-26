package frc.robot;

import frc.robot.autos.TestAuto;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.claw.Pistons;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.controls.deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Flipper flipper = new Flipper();
  private final Arm arm = new Arm();
  private final Pistons pistons = new Pistons();


  public RobotContainer() {

    configureBindings();
  
  }

 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftY(), 0.05) * 3, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftX(), 0.05) * 3 , 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getRightX(), 0.05) * 3 , 
        true));
       
    

      mOperator.y().whileTrue(
        new InstantCommand(
          pistons::extend,
          pistons
        )
      );

      mOperator.x().whileTrue(
        new InstantCommand(
          pistons::retract,
          pistons
        )
      );
      
      mOperator.b().whileTrue(
        new InstantCommand(
          () -> arm.run(-0.2)
        )
      );

      mDriver.a().whileTrue(
        new StartEndCommand(
          () -> flipper.run(-0.25),
          flipper::stop,
          flipper
        )
      );

      // Resets the gyro, specifically for when field relative driving is on
      mDriver.x().whileTrue(
        new InstantCommand(
          drivetrain::resetGyro, 
          drivetrain)
      );

    }

  
  public Command getAutonomousCommand(){

    return new TestAuto(drivetrain);

  } 
   

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  }
