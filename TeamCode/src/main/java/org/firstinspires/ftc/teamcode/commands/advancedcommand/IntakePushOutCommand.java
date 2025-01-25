package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class IntakePushOutCommand extends SequentialCommandGroup {
    public IntakePushOutCommand(int ext) {
        super(
                new InstantCommand(Robot.getInstance().data::startIntaking),
                new ExtensionPositionCommand(ext),
                new ArmStateCommand(IntakeSubsystem.ArmState.INTAKE),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
        );
    }

    public IntakePushOutCommand(int ext, boolean intake) {
        super(
                new InstantCommand(Robot.getInstance().data::startIntaking),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new IntakePushStateCommand(IntakeSubsystem.IntakePushState.DRIVE),
                                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
                        ),
                        new SequentialCommandGroup(
                                new IntakePushStateCommand(IntakeSubsystem.IntakePushState.PUSH)
                        ),
                        () -> intake
                ),
                new ExtensionPositionCommand(ext),
                new ArmStateCommand(IntakeSubsystem.ArmState.INTAKE)

        );
    }
}
