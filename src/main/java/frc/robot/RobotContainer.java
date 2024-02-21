package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.HeadingControl;
import frc.robot.utils.SwerveModule;
import frc.robot.subsystems.Intake;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final CommandXboxController driver;

  private Intake intake = new Intake();

  public final Swerve swerve;

  // public final AutoCommands auto;

  public RobotContainer() {
    driver = new CommandXboxController(Constants.kControls.DRIVE_JOYSTICK_ID);

    swerve = new Swerve();

    this.intake.setDefaultCommand(new RunCommand(intake::off, intake));

    SmartDashboard.putNumber("drive/speed", 0.0);
    SmartDashboard.putNumber("drive/velocity(RPM)", 0.0);
    // auto = new AutoCommands(swerve);

    // Configure button bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(swerve.drive(
      () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS)),
      () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS)), 
      () -> -Constants.kControls.THETA_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.ROTATION_AXIS)),
      true,
      true
    ));


    driver.a().whileTrue(swerve.drive(
      () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS)),
      () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS)), 
      () -> Rotation2d.fromDegrees(0.0),
      true,
      true
    ));


    driver.b().whileTrue(swerve.drive(
      () -> SmartDashboard.getNumber("drive/speed", 0.0),
      () -> 0.0, 
      () -> 0.0,
      true,
      true
    ));

    driver.y().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())));
  }

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoName = "Auto1";
    Command resetOdometry = new InstantCommand(() -> 
        swerve.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoName)));
    return resetOdometry.andThen(new PathPlannerAuto(autoName));
  }
}
