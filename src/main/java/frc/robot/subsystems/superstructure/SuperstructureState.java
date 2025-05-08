// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;
import frc.robot.subsystems.superstructure.SuperstructureStateData.Height;
import frc.robot.subsystems.superstructure.manipulator.Manipulator;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
  START(SuperstructureStateData.builder().build()),
  AUTO_START(SuperstructureStateData.builder().build()),
  CHARACTERIZATION(SuperstructureStateData.builder().build()),
  SAFETY(SuperstructureStateData.builder().build()),
  STOWREST(SuperstructureStateData.builder().pose(Preset.STOWREST.getPose()).build()),
  STOWTRAVEL(SuperstructureStateData.builder().pose(Preset.STOWTRAVEL.getPose()).build()),
  CORAL_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.CORAL_INTAKE.getPose())
          .height(Height.BOTTOM)
          .MailboxGoal(Manipulator.MailboxGoal.CORALINTAKE)
          .build()),
  L2_CORAL(
      SuperstructureStateData.builder()
          .height(Height.FIRST_STAGE)
          .pose(Preset.L2.getPose())
          .build()),
  L3_CORAL(
      SuperstructureStateData.builder()
          .height(Height.FIRST_STAGE)
          .pose(Preset.L3.getPose())
          .build()),
  L4_CORAL(
      SuperstructureStateData.builder()
          .height(Height.SECOND_STAGE)
          .pose(Preset.L4.getPose())
          .MailboxGoal(Manipulator.MailboxGoal.CORALL4GRIP)
          .build()),
  L2_CORAL_EJECT(
      L2_CORAL.getValue().toBuilder().MailboxGoal(Manipulator.MailboxGoal.CORALEJECT).build()),
  L3_CORAL_EJECT(
      L3_CORAL.getValue().toBuilder().MailboxGoal(Manipulator.MailboxGoal.CORALEJECT).build()),
  L4_CORAL_EJECT(
      L4_CORAL.getValue().toBuilder().MailboxGoal(Manipulator.MailboxGoal.CORALEJECT).build()),
  ALGAE_STOW(
      SuperstructureStateData.builder()
          .pose(Preset.PROCESS.getPose())
          .MailboxGoal(Manipulator.MailboxGoal.ALGAEHOLD)
          .build()),
  ALGAE_L2_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L2_INTAKE.getPose())
          .height(Height.FIRST_STAGE)
          .MailboxGoal(Manipulator.MailboxGoal.ALGAEGRAB)
          .build()),
  ALGAE_L3_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L3_INTAKE.getPose())
          .height(Height.SECOND_STAGE)
          .MailboxGoal(Manipulator.MailboxGoal.ALGAEGRAB)
          .build()),
  PRE_THROW(
      SuperstructureStateData.builder()
          .pose(Preset.THROW.getPose())
          .height(Height.SECOND_STAGE)
          .MailboxGoal(Manipulator.MailboxGoal.ALGAEHOLD)
          .build()),
  THROW(PRE_THROW.getValue().toBuilder().MailboxGoal(Manipulator.MailboxGoal.ALGAENET).build()),
  TOSS(
      SuperstructureState.ALGAE_STOW.getValue().toBuilder()
          .MailboxGoal(Manipulator.MailboxGoal.ALGAENET)
          .build()),
  PRE_PROCESS(
      SuperstructureStateData.builder()
          .pose(Preset.PROCESS.getPose())
          .MailboxGoal(Manipulator.MailboxGoal.ALGAEHOLD)
          .build()),
  PROCESS(
      SuperstructureState.PRE_PROCESS.getValue().toBuilder()
          .MailboxGoal(Manipulator.MailboxGoal.ALGAENET)
          .build());

  private final SuperstructureStateData value;
}
