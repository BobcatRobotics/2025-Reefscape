// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Limelight;

public class AprilTagVisionConstants {
  // public static final double ambiguityThreshold = 0.4;
  // public static final double targetLogTimeSecs = 0.1;
  // public static final double fieldBorderMargin = 0.5;
  // public static final double zMargin = 0.75;
  // public static final double xyStdDevCoefficient = 0.005;
  // public static final double thetaStdDevCoefficient = 0.01;

  //   public static final double[] stdDevFactors =new double[] {1.0, 1.0};

  //   public static final Pose3d[] cameraPoses =
  //             new Pose3d[] {
  //               new Pose3d(
  //                   -1*Units.inchesToMeters(6.5),
  //                   Units.inchesToMeters(0),
  //                   Units.inchesToMeters(10),
  //                   new Rotation3d(0.0, Units.degreesToRadians(-30),
  // Units.degreesToRadians(180.0))),
  //                       // .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))),
  //               new Pose3d(
  //                   Units.inchesToMeters(0),
  //                   Units.inchesToMeters(0),
  //                   Units.inchesToMeters(0),
  //                   new Rotation3d(0.0, Units.degreesToRadians(0), 0.0)
  //                       .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))))
  //             };

  //   public static final String[] instanceNames =
  //         new String[] {"northstar0", "northstar1"};

  //   public static final String[] cameraIds =
  //             new String[] {
  //               "/dev/video0",
  //               "cam1"
  //             };

  public class limelightConstants {
    public static final double rotationTolerance = 10000;
    public static final double throwoutDist = 4.5;
    public static final double xySingleTagStdDev = 0.6;
    public static final double thetaSingleTagStdDev = 9999999;
    public static final double xyMultiTagStdDev = 0.45;
    public static final double thetaMultiTagStdDev = 99999999;
    public static final int[] validTags =
        new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}; // No barge tags
  }
}
