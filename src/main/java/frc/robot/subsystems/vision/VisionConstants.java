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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String frontCam = "limelight-front";
    public static String leftCam = "limelight-left";
    public static String rightCam = "limelight-right";

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double frontLinearStdDevBaseline = 0.02; // Meters
    public static double frontAngularStdDevBaseline = 0.06; // Radians

    public static double leftLinearStdDevBaseline = 0.04; // Meters
    public static double leftAngularStdDevBaseline = 0.12; // Radians

    public static double rightLinearStdDevBaseLine = 0.04; // Meters
    public static double rightAngularStdDevBaseLine = 0.12; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0, 1.0, 1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    //yellow cone = turret cam
    public static Transform3d frontCamTrans = new Transform3d(
            new Translation3d(0.24, -0.42, 0.47), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(270)));

    //green = left cam
    public static Transform3d leftCamTrans = new Transform3d(
            new Translation3d(0.3429, .35, 0.5715), new Rotation3d(0, Math.toRadians(-15), Math.toRadians(45)));

    //red = forward cam
    public static Transform3d rightCamTrans = new Transform3d(
            new Translation3d(.2, 0.2, 0.4445),
            new Rotation3d(Math.toRadians(180), Math.toRadians(-15), Math.toRadians(180)));
}
