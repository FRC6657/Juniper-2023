package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flipper;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final Brake brake = new Brake();



  public RobotContainer() {

    LiveWindow.disableAllTelemetry();

    configureBindings();
  

  }

 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftY(), 0.05) * 3, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftX(), 0.05) * 3 , 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getRightX(), 0.05) * 3 , 
        true));

      // CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      //   new DriverControl(drivetrain, 
      //     ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftY(), 0.05), 
      //     ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftX(), 0.05), 
      //     ()-> deadbander.applyLinearScaledDeadband(-mDriver.getRightX(), 0.05)));
       
    

      mOperator.y().toggleOnTrue(
        new InstantCommand(
          brake::extend,
          brake
        )
      );
      
      mDriver.b().whileTrue(
        new StartEndCommand(
          () -> flipper.run(0.25),
          flipper::stop,
          flipper
        )
      );

      mDriver.a().whileTrue(
        new StartEndCommand(
          () -> flipper.run(-0.25),
          flipper::stop,
          flipper
        )
      );

      mDriver.x().whileTrue(
        new InstantCommand(
          drivetrain::resetGyro, 
          drivetrain)
      );

      // mOperator.x().whileTrue(
      //   new StartEndCommand(
      //     claw::foward, 
      //     claw::stop, 
      //     claw)
      // );

      mOperator.x().whileTrue(
        new RunCommand(
          claw::foward,
          claw
        )
      );

      mOperator.y().whileTrue(
        new StartEndCommand(
          claw::reverse, 
          claw::stop, 
          claw)
      );

    }

  
  public Command getAutonomousCommand(){

    return new DriverControl(
      drivetrain, 
      () -> 0.0, 
      () -> 1, 
      () -> 0.0, 
      false);

  }    


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  }
