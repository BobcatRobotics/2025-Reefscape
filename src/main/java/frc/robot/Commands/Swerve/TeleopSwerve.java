package frc.robot.Commands.Swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Assists.AimAssist;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve swerve;
    private DoubleSupplier translation;
    private DoubleSupplier fineStrafe;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private BooleanSupplier robotCentric;
    private DoubleSupplier fineTrans;
    private AimAssist assist;

    /**
     * suppliers are objects that can return a value that will change, for example a
     * method that returns a double can be input as a doubleSupplier
     * 
     * @param swerve
     * @param translation
     * @param strafe
     * @param rotation
     * @param robotCentric
     * @param fineStrafe
     * @param fineTrans
     * @param snapToAmp
     * @param snapToSpeaker
     * @param pass
     * @param ampAssist
     * @param rotateToNote
     */
    public TeleopSwerve(
            Swerve swerve,
            DoubleSupplier translation,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            BooleanSupplier robotCentric,
            DoubleSupplier fineStrafe,
            DoubleSupplier fineTrans,
            AimAssist assist) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.assist = assist;

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.robotCentric = robotCentric;
        this.fineStrafe = fineStrafe;
        this.fineTrans = fineTrans;
    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConstants.stickDeadband); // from 0 to one

        /*
         * If joysticks not receiving any normal input, use twist values for fine adjust
         */
        if (strafeVal == 0.0) {
            strafeVal = fineStrafe.getAsDouble();
        }
        if (translationVal == 0.0) {
            translationVal = fineTrans.getAsDouble();
        }


        Logger.recordOutput("Swerve/DesiredTranslation",
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed));
        /* Drive */
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed),
                rotationVal * SwerveConstants.maxModuleAngularVelocity,
                !robotCentric.getAsBoolean(),
                assist);
    }
}