package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.HeadingControl;
import frc.robot.utils.SwerveModule;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules;

  private final AHRS gyro;

  private final HeadingControl m_headingController = new HeadingControl();

  private final SwerveDriveOdometry m_odometry;

  private final Field2d m_dashboardField = new Field2d();

  private final List<DoubleLogEntry> m_moduleRawVelocityEntries = new ArrayList<DoubleLogEntry>();
  private final List<DoubleLogEntry> m_modulepercentPowerEntries = new ArrayList<DoubleLogEntry>();


  public Swerve() {
    gyro = new AHRS();

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };

    DataLog log = DataLogManager.getLog();
    for (int i = 0; i < 4; i++) {
      m_moduleRawVelocityEntries.add(new DoubleLogEntry(log, "/swerve/moduleRawVelocity[" + i + "]"));
      m_modulepercentPowerEntries.add(new DoubleLogEntry(log, "/swerve/moduleOutPower[" + i + "]"));
    }

    zeroGyro();

    m_odometry = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getYaw(), getModulePositionStates());

    SmartDashboard.putData("Field", m_dashboardField);

    SmartDashboard.putBoolean("put k vals", false);
    SmartDashboard.putNumber("drivekP", Constants.kSwerve.ANGLE_KP);
    SmartDashboard.putNumber("drivekD", Constants.kSwerve.ANGLE_KD);


    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this::setFlipPath,
        this // Reference to this subsystem to set requirements
    );
  }

  /** 
   * This is called a command factory method, and these methods help reduce the
   * number of files in the command folder, increasing readability and reducing
   * boilerplate. 
   * 
   * Double suppliers are just any function that returns a double.
   */
  public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isFieldRelative, boolean isOpenLoop) {
    return new RunCommand(() -> {
      // Grabbing input from suppliers.
      double forwardBack = forwardBackAxis.getAsDouble();
      double leftRight = leftRightAxis.getAsDouble();
      double rotation = rotationAxis.getAsDouble();

      m_headingController.update(rotation, m_odometry.getPoseMeters().getRotation());

      // Adding deadzone.
      forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
      rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

      // Converting to m/s
      forwardBack *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND; 
      leftRight *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      rotation *= Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;

      if (m_headingController.getIfSnapping()) {
        setDrive(forwardBack, leftRight, m_headingController.getSnapHeading(), isFieldRelative, isOpenLoop);
      } else {
        setDrive(forwardBack, leftRight, rotation, isFieldRelative, isOpenLoop);
      }
    }, this).withName("SwerveDriveCommand");
  }

  public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, Supplier<Rotation2d> headingAngle, boolean isFieldRelative, boolean isOpenLoop) {
    return new RunCommand(() -> {
      // Grabbing input from suppliers.
      double forwardBack = forwardBackAxis.getAsDouble();
      double leftRight = leftRightAxis.getAsDouble();
      Rotation2d heading = headingAngle.get();

      // Adding deadzone.
      forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;

      // Converting to m/s
      forwardBack *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND; 
      leftRight *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;

      setDrive(forwardBack, leftRight, heading, isFieldRelative, isOpenLoop);
    }, this).withName("SwerveDriveCommand");
  }

  private void setDrive(double x_mps, double y_mps, double rotation_rps, boolean isFieldRelative, boolean isOpenLoop) {
      // Get desired module states.
      ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(x_mps, y_mps, rotation_rps, m_odometry.getPoseMeters().getRotation())
        : new ChassisSpeeds(x_mps, y_mps, rotation_rps);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);
  }

  private void setDrive(double x_mps, double y_mps, Rotation2d heading, boolean isFieldRelative, boolean isOpenLoop) {
      PIDController headingController = new PIDController(3.0, 0.0, 0.0); // if 'ki' or 'kd' is needed, make a member variable
      headingController.enableContinuousInput(-Math.PI, Math.PI);
      double rotation = headingController.calculate(m_odometry.getPoseMeters().getRotation().getRadians(), heading.getRadians());
      //System.out.println(rotation);
      headingController.close();

      setDrive(x_mps, y_mps, rotation, isFieldRelative, isOpenLoop);
  }

  public void driveRobotRelative(ChassisSpeeds speed) {
    double forwardBack = speed.vxMetersPerSecond;
    double leftRight = speed.vyMetersPerSecond;
    double rotation = speed.omegaRadiansPerSecond;

    // Get desired module states.
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardBack, leftRight, rotation);
    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(states, false);
  }

  public boolean setFlipPath(){
    //this function needs to be updated to allow flipping between RED & BLUE sides
    BooleanSupplier sup = () -> true;
    return sup.getAsBoolean();
  }

  private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the module states don't exceed the max speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
    }
  }

  public SwerveModulePosition[] getModulePositionStates() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getModulePosition();
    }
    return currentStates;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getModuleState();
    }
    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }

  public void resetOdometry(Pose2d resetPose) {
    m_odometry.resetPosition(getYaw(), getModulePositionStates(), resetPose);
    m_headingController.reset(getYaw());
  }

  public Command zeroGyroCommand() {
    return new InstantCommand(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.kSwerve.KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {
    m_odometry.update(getYaw(), getModulePositionStates());
    m_dashboardField.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("x_val odom", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y_val odom", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("angle odom", m_odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("navX", gyro.getAngle());

    SmartDashboard.putBoolean("Heading Controller", m_headingController.getIfSnapping());

    int i = 0;

    for (SwerveModule module : modules) {
      SmartDashboard.putNumber(String.format("Thrifty angle %d", module.moduleNumber), module.getThriftyAngle().getDegrees());
      SmartDashboard.putNumber(String.format("Max angle %d", module.moduleNumber), module.getSteerAngle().getDegrees());
      SmartDashboard.putNumber(String.format("Distance %d", module.moduleNumber), module.getDisance());
      SmartDashboard.putNumber(String.format("Rot %d", module.moduleNumber), module.getDriveRot());

      m_moduleRawVelocityEntries.get(i).append(module.getDriveRawVelocity());
      m_modulepercentPowerEntries.get(i).append(module.getDriveOutputPower());
      i++;
      
      SmartDashboard.putNumber(String.format("percentPower %d", module.moduleNumber), module.getDriveOutputPower());
      SmartDashboard.putNumber(String.format("rawVelocity %d", module.moduleNumber), module.getDriveRawVelocity());
      SmartDashboard.putNumber(String.format("driveError %d", module.moduleNumber), Math.abs(module.getDriveError()));

      SmartDashboard.putNumber(String.format("inchVelocity %d", module.moduleNumber), Units.metersToInches(module.getDriveVelocity()));
      SmartDashboard.putNumber(String.format("inchError %d", module.moduleNumber), Units.metersToInches(Math.abs(module.getVeolcityError())));
    }
   
    if (SmartDashboard.getBoolean("put k vals", false)) {
    double kP = SmartDashboard.getNumber("drivekP", Constants.kSwerve.ANGLE_KP);
    double kD = SmartDashboard.getNumber("drivekD", Constants.kSwerve.ANGLE_KD);
      for (SwerveModule module : modules) {
        module.setPID(kP, 0, kD);
      }
    } 
  }
  
//   public Command followPathCommand(String pathName){
//     PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

//     // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
//     return new FollowPathWithEvents(
//         new FollowPathHolonomic(
//             path,
//             this::getPose, // Robot pose supplier
//             this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//             this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//             new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//                 1.0, // Max module speed, in m/s
//                 0.6, // Drive base radius in meters. Distance from robot center to furthest module.
//                 new ReplanningConfig() // Default path replanning config. See the API for the options here
//             ),
//             this // Reference to this subsystem to set requirements
//         ),
//         path, // FollowPathWithEvents also requires the path
//         this::getPose // FollowPathWithEvents also requires the robot pose supplier
//     );
// }


}
