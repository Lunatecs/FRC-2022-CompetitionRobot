// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TriggerButton extends Trigger {
    DoubleSupplier axis;
    public TriggerButton(DoubleSupplier axis) {
        this.axis = axis;
    }

    @Override
    public boolean get() {
        if (Math.abs(axis.getAsDouble()) > .2) {
            return true;
        } else {
            return false;
        }
    }
}
