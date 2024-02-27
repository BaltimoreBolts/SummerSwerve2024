package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
  public double power = 0;

  private CANSparkMax intakeMotor;
  
  //private RelativeEncoder intakeMotorEncoder;

  private DigitalInput noteAtIntake;
  private DigitalInput noteAtShooter;
  
  public void intake(){

    this.intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    this.intakeMotor.restoreFactoryDefaults();
    this.intakeMotor.setSmartCurrentLimit(40);
    this.intakeMotor.burnFlash();
    //intakeMotorEncoder = intakeMotor.getEncoder();

    noteAtIntake = new DigitalInput(0);
    noteAtShooter = new DigitalInput(1);
  }

  final double intakeSpeed = 0.5;
  final double outakeSpeed = 0.5;

  @Override
  public void periodic() {
    this.intakeMotor.set(power);
  }
  
  public Command intakeOffCommand() {
    return new InstantCommand(() -> intakeOff());
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> outtake());
  }
  public Command intakeFastCommand() {
    return new InstantCommand(() -> intakeFast());
  }
  public BooleanSupplier seeShooterSupplier() {
    return noteAtShooter::get;
  }
  private void intakeOff() {
    this.power = 0;
  }
  private void outtake() {
    this.power = -outakeSpeed;
  }
  private void intakeFast() {
    this.power = intakeSpeed;
  }

}
