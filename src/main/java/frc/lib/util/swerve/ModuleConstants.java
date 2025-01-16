package frc.lib.util.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final Slot0Configs driveSlot0;
    public final Slot0Configs angleSlot0;
    public final boolean driveInverted;
    public final boolean angleInverted;
    public final boolean openLoopTurn;
    public final boolean openLoopDrive;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public ModuleConstants(
        int driveMotorID, 
        int angleMotorID,
        int canCoderID,
        Rotation2d angleOffset,
        Slot0Configs driveSlot0,
        Slot0Configs angleSlot0,
        boolean driveInverted,
        boolean angleInverted,
        boolean openLoopTurn,
        boolean openLoopDrive) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.driveSlot0 = driveSlot0;
        this.angleSlot0 = angleSlot0;
        this.driveInverted = driveInverted;
        this.angleInverted = angleInverted;
        this.openLoopTurn = openLoopTurn;
        this.openLoopDrive = openLoopDrive;


    }
}
