package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommands extends SequentialCommandGroup{
    public IntakeCommands(Intake intake) {
            addCommands(new ParallelRaceGroup(new RunCommand(() -> {
                    intake.intakeFast();
            }), new WaitCommand(5.0)), new InstantCommand(intake::off, intake));

            addRequirements(intake);
        }
}

