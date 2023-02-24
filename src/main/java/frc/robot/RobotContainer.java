package frc.robot;

import frc.robot.autos.TaxiChargeBlue;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Pistons;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.controls.deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);
  private CommandXboxController mTesting = new CommandXboxController(2);

  private static final Field2d mField = new Field2d();

  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final Pistons pistons = new Pistons();
  private final Claw claw = new Claw();
  private final Brake brake = new Brake();
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
    
        mTesting.x().whileTrue(
          new InstantCommand(
            brake::extend,
            brake
          )
        );

        mTesting.y().whileTrue(
          new InstantCommand(
            brake::retract,
            brake
          )
        );

      //Cube - extend, wheels spin
      mOperator.b().whileTrue(
        new SequentialCommandGroup(
          new InstantCommand(
            pistons::extend,
            pistons
          ),
          new InstantCommand(
            claw::intake,
            claw
          )
          )).whileFalse(
            new SequentialCommandGroup(
              new InstantCommand(
                claw::stop,
                claw),
              new InstantCommand(
                pistons::retract, 
                pistons)
            )
        );

      //Cone - extend
      mOperator.a().whileTrue(
        new InstantCommand(
          pistons::extend,
          pistons
          )
      ).whileFalse(
        new InstantCommand(
          pistons::retract,
          pistons
        )
      );

      mOperator.rightBumper().whileTrue(
        new InstantCommand(
          pistons::extend,
          pistons
        )
      ).whileFalse(
          new InstantCommand(
            pistons::retract, 
            pistons)
      );

      mOperator.povUp().whileTrue(
        new InstantCommand(
          pivot::forward,
          pivot)
      ).whileFalse(
        new InstantCommand(
          pivot::stop,
          pivot
        )
      );

      mOperator.povDown().whileTrue(
        new InstantCommand(
          pivot::backward,
          pivot)
      ).whileFalse(
        new InstantCommand(
          pivot::stop,
          pivot
        )
      );

      mOperator.leftTrigger().whileTrue(
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
      
      mOperator.rightTrigger().whileTrue(
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

      //Driver
      // Resets the gyro, specifically for when field relative driving is on
      mDriver.x().whileTrue(
        new InstantCommand(
          drivetrain::resetGyro, 
          drivetrain)
      );

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

      }

      public Command getAutonomousCommand() {
         return new TaxiChargeBlue(drivetrain);
      }

      public static Field2d getField() {
        return mField;
      }

      //To run when disabled
      public Command stopAll() {
        return new ParallelCommandGroup(
          new InstantCommand(pivot::stop),
          new InstantCommand(arm::stop),
          new InstantCommand(drivetrain::stop),
          new InstantCommand(claw::stop)
        );
      };
    }