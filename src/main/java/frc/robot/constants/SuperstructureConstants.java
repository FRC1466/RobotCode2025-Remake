// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

public class SuperstructureConstants {
  public static final double xOffsetFromTagForL1BaseScoringInches = 21.0;
  public static final double xOffsetFromTagForL1TopScoringInches = 19.25;

  public static final double xOffsetFromTagForScoringInches = 22.0;
  public static final double xOffsetFromTagForIntakingAlgaeInches = 18.0;
  public static final double xOffsetFromTagForIntermediateIntakingAlgaeInches = 30.0;
  public static final double xOffsetFromTagForBackoutIntakingAlgaeInches = 50.0;
  public static final double xOffsetFromTagForL1BackoutInches = 10.0;

  public static final double yOffsetFromTagForScoringOnReefInches = 6.5;
  public static final double yOffsetFromTagForScoringL1Inches = 9.0;

  public enum ScoringSide {
    RIGHT,
    LEFT
  }

  public enum ReefSelectionMethod {
    POSE,
    ROTATION
  }
}
