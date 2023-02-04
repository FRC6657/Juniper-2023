package frc.robot;

import frc.robot.autos.TestAuto;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.claw.Pistons;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.controls.deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);
  private CommandXboxController mTesting = new CommandXboxController(2);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final Pistons pistons = new Pistons();
  private final Pivot pivot = new Pivot();


  public RobotContainer() {

    configureBindings();
  
  }

 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftY(), 0.1) * 3, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftX(), 0.1) * 3 , 
        ()-> deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.1) * 3, 
        false));

        mTesting.a().toggleOnTrue(
          new InstantCommand(
            pivot::ratchetEnable,
            pivot
          )
        );

        mTesting.b().toggleOnTrue(
          new InstantCommand(
            pivot::ratchetDisable,
            pivot
          )
        );

        mTesting.x().whileTrue(
          new InstantCommand(
            pistons::extend,
            pistons
          )
        );

        mTesting.y().whileTrue(
          new InstantCommand(
            pistons::retract,
            pistons
          )
        );

      //Pivot upward
      mOperator.a().whileTrue(
        new SequentialCommandGroup(
          new InstantCommand(
            pivot::ratchetEnable,
            pivot),  
          new InstantCommand(
            pivot::forward,
            pivot)
        )
      ).whileFalse(
          new InstantCommand(
            pivot::stop,
            pivot)
          );

      mOperator.b().whileTrue(
        new SequentialCommandGroup(
          new InstantCommand(
            pivot::ratchetDisable, 
            pivot),
          new InstantCommand(
            pivot::backward,
            pivot)
        )
      ).whileFalse(
        new SequentialCommandGroup(
          new InstantCommand(
            pivot::stop, 
            pivot),
          new InstantCommand(
            pivot::ratchetEnable,
            pivot)
        )
      );

      mOperator.x().whileTrue(
        new InstantCommand(
          arm::retract,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );
      
      mOperator.y().whileTrue(
        new InstantCommand(
          arm::extend,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
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
