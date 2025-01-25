// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.limelight.limelightConstants;
import frc.robot.util.VisionObservation;
import frc.robot.util.VisionObservation.LLTYPE;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class limelight {

    public static final String name = "limelight";
    public static final VisionObservation.LLTYPE limelightType = LLTYPE.LL3G;
    public static final double verticalFOV = 49.7; // degrees obviously
    public static final double horizontalFOV = 63.3;
    public static final double limelightMountHeight = Units.inchesToMeters(20.5);
    public static final int detectorPiplineIndex = 2;
    public static final int apriltagPipelineIndex = 1;
    public static final int horPixles = 1280;
    public static final double filterTimeConstant =
        0.1; // in seconds, inputs occuring over a time period
    // significantly shorter than this will be thrown out
    public static final Vector<N3> visionMeasurementStdDevs =
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    public static final int movingAverageNumTaps = 20;

    public static final limelightConstants constants =
        new limelightConstants(
            name,
            limelightType,
            verticalFOV,
            horizontalFOV,
            limelightMountHeight,
            detectorPiplineIndex,
            apriltagPipelineIndex,
            horPixles,
            visionMeasurementStdDevs);

    public static final String ip = "10.1.77.11";
  }
}
