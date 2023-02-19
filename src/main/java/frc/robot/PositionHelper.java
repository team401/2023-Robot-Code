package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public final class PositionHelper {
    
    public static double[] getDouble(Position position, GamePieceMode mode) {
        switch(position) {
            case Ground:
                return getGround(mode);
            case Mid:
                return getMid(mode);
            case High:
                return getHigh(mode);
            case Shelf:
                return getShelf(mode);
            default:
                return ArmPositions.stow;
        }
    }

    private static double[] getGround(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.intakeCubeGround;
        if (mode == GamePieceMode.ConeBack) return ArmPositions.intakeConeBackGround;
        return null;
    }

    private static double[] getMid(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.placeCubeMid;
        if (mode == GamePieceMode.ConeBack) return ArmPositions.placeConeBackMid;
        return null;
    }
    
    private static double[] getHigh(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.placeCubeHigh;
        if (mode == GamePieceMode.ConeBack) return ArmPositions.placeConeBackHigh;
        return null;
    }

    private static double[] getShelf(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.intakeCubeShelf;
        if (mode == GamePieceMode.ConeBack) return ArmPositions.intakeConeBackShelf;
        return null;
    }
}
