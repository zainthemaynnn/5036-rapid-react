// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OrElseCommand implements Command {
  private Command y, n, curr;
  private BooleanSupplier cond;
  private Set<Subsystem> requirements = new HashSet<>();

  public OrElseCommand(Command y, Command n, BooleanSupplier cond) {
    this.y = y;
    this.n = n;
    this.cond = cond;
    requirements.addAll(y.getRequirements());
    requirements.addAll(n.getRequirements());
  }

  private Command getYN() {
    return cond.getAsBoolean() ? y : n;
  }

  @Override
  public void initialize() {
    curr = getYN();
  }

  @Override
  public void execute() {
    var cmd = getYN();
    if (!curr.equals(cmd)) {
      curr.cancel();
      curr = cmd;
    }
    cmd.execute();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }
}
