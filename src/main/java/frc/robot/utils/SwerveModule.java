package frc.robot.utils;

//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.SensorInitializationStrategy;
//import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.AnalogPotentiometer; // FOR ANALOG ENCODER (Maybe)
import edu.wpi.first.wpilibj.AnalogEncoder; // FOR ANALOG ENCODER
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;


public class SwerveModule {
  public final int moduleNumber;

  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkPIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;

  private final CANSparkMax angleMotor;
  private final RelativeEncoder angleEncoder;
  private final SparkPIDController anglePID;
  
  private final AnalogEncoder m_turningEncoder; // FOR ANALOG ENCODER
  private final double m_thriftyOffsetDegrees;

  private double m_startupOffset; // TODO move later
  
  private static final double k_turnGearRatio = 7.0/150.0;
  
  //private final CANCoder canCoder;
  //private final double canCoderOffsetDegrees;


  private double lastAngle;

  public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getPIDController();
    driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    angleMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();
    anglePID = angleMotor.getPIDController();

    m_turningEncoder = new AnalogEncoder(constants.thriftyEncoderID);
    m_thriftyOffsetDegrees = constants.thriftyOffsetDegrees;
    
    //canCoder = new CANCoder(constants.canCoderID);
    //canCoderOffsetDegrees = constants.canCoderOffsetDegrees;

    configureDevices();
    lastAngle = getState().angle.getRadians();
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    state = SwerveModuleState.optimize(state, getState().angle);

    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      drivePID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    } else {
      drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, driveFeedforward.calculate(state.speedMetersPerSecond));
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
      ? lastAngle
      : state.angle.getRadians();

    anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  public double getSwerveAngle() {
    // return (m_turningEncoder.getAbsolutePosition()* 2.0 * Math.PI); // should be outputing # between 0-1*2pi
    return ((Units.rotationsToRadians(angleMotor.getEncoder().getPosition()) * k_turnGearRatio) + m_startupOffset);
  }

  public double getSwerveRawAngle() {
    // return (m_turningEncoder.getAbsolutePosition()* 2.0 * Math.PI); // should be outputing # between 0-1*2pi
    return (Units.rotationsToRadians(angleMotor.getEncoder().getPosition()) * k_turnGearRatio);
  }

  private double getThriftyAngle() {
    return Units.rotationsToRadians(m_turningEncoder.getAbsolutePosition());
  }

  public Rotation2d getAngle() {
    return new Rotation2d(angleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  private void configureDevices() {
    // Drive motor configuration.
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(Constants.kSwerve.DRIVE_MOTOR_INVERSION);
    driveMotor.setIdleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    driveMotor.setOpenLoopRampRate(Constants.kSwerve.OPEN_LOOP_RAMP);
    driveMotor.setClosedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
    driveMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);
 
    drivePID.setP(Constants.kSwerve.DRIVE_KP);
    drivePID.setI(Constants.kSwerve.DRIVE_KI);
    drivePID.setD(Constants.kSwerve.DRIVE_KD);
    drivePID.setFF(Constants.kSwerve.DRIVE_KF);
 
    driveEncoder.setPositionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
    driveEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    driveEncoder.setPosition(0);

    // Angle motor configuration.
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION);
    angleMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    angleMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

    anglePID.setP(Constants.kSwerve.ANGLE_KP);
    anglePID.setI(Constants.kSwerve.ANGLE_KI);
    anglePID.setD(Constants.kSwerve.ANGLE_KD);
    anglePID.setFF(Constants.kSwerve.ANGLE_KF);

    anglePID.setPositionPIDWrappingEnabled(true);
    anglePID.setPositionPIDWrappingMaxInput(1.0 / k_turnGearRatio);
    anglePID.setPositionPIDWrappingMinInput(0.0);

    angleEncoder.setPositionConversionFactor(1.0);
    angleEncoder.setVelocityConversionFactor(1.0);
    //angleEncoder.setPosition(getThriftyAngle() - Units.degreesToRadians(m_thriftyOffsetDegrees));
    configureEncoders();
  }

  public void configureEncoders() {
    System.out.println("Start");
    System.out.println(getThriftyAngle());
    System.out.println(m_thriftyOffsetDegrees);
    System.out.println(Units.degreesToRadians(m_thriftyOffsetDegrees));
    System.out.println(getThriftyAngle() - Units.degreesToRadians(m_thriftyOffsetDegrees));
    System.out.println(getSwerveRawAngle());
    // angleEncoder.setPosition(Units.radiansToRotations(getThriftyAngle() - Units.degreesToRadians(m_thriftyOffsetDegrees)));
    m_startupOffset = (getThriftyAngle() - Units.degreesToRadians(m_thriftyOffsetDegrees)) - getSwerveRawAngle();
  }

  public void update() {
    SmartDashboard.putNumber("Thrifty Encoder", Units.radiansToDegrees(getThriftyAngle()));
    SmartDashboard.putNumber("Steer Spark Encoder",  Units.radiansToDegrees(getSwerveAngle()));
    if (RobotController.getUserButton()) {
      configureEncoders();
    }
    
  }

  public void setAngle(double angle_rad) {
    double position = Units.radiansToRotations((angle_rad - m_startupOffset) / k_turnGearRatio);
    SmartDashboard.putNumber("position given", position);
    SmartDashboard.putNumber("position given 2", angle_rad);
    anglePID.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    angleMotor.set(0);
  }


}
