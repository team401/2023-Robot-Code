package frc.robot.util;

import java.util.HashMap;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;

public class AutoManager {

    private PivotSubsystem pivot; 
    private TelescopeSubsystem telescope; 
    private WristSubsystem wrist; 
    private Drive drive; 
    private IntakeSubsystem intake;
    private HashMap<String, Command> eventMap;
    private Function<Position, Command> getMoveCommand;

    public AutoManager(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist, Drive drive, IntakeSubsystem intake, Function<Position, Command> function) {
        this.pivot = pivot; this.telescope = telescope; this.wrist = wrist; this.drive = drive; this.intake = intake; getMoveCommand = function;

        eventMap = new HashMap<String, Command>() {{
            put("WaitUntilHomed", new WaitUntilCommand(() -> (wrist.homed && telescope.homed)));
            put("PlaceCube", new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
                getMoveCommand.apply(Position.High),
                new InstantCommand(intake::place),
                new WaitCommand(0.25),
                new InstantCommand(intake::stopMotor)
            ));
            put("PlaceCone", new SequentialCommandGroup(
                new MoveWrist(wrist, pivot, () -> ArmPositions.wristConePlace),
                new InstantCommand(intake::stopMotor)
            ));
            put("PreparePlaceCone", new InstantCommand(
                () -> getMoveCommand.apply(Position.High).schedule()
            ));
            put("PickupCone", new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeBack)),
                new InstantCommand(() -> getMoveCommand.apply(Position.Ground).schedule()),
                new InstantCommand(intake::intake)
            ));
            put("Stow", new InstantCommand(
                () -> getMoveCommand.apply(Position.Stow).schedule()
            ));
            put("Invert", new InstantCommand(
                () -> RobotState.getInstance().invertBack()
            ));
            put("Balance", new Balance(drive));
        }};
    }

    public HashMap<String, Command> getEventMap() {
        return eventMap;
    }


    
}
