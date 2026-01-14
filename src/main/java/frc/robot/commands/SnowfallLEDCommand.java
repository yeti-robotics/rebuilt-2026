// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;

public class SnowfallLEDCommand extends Command {
    /** Creates a new SnowfallLEDCommand. */
    private final LED ledSubsystem;

    long waitTime;
    long startTime;
    int stage;

    public SnowfallLEDCommand(LED ledSubsystem, long waitTime) {
        this.ledSubsystem = ledSubsystem;
        this.waitTime = waitTime;
        this.startTime = System.currentTimeMillis();
        stage = 0;
        addRequirements(ledSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
            if (i % 4 == stage) {
                ledSubsystem.setRGB(i, 255, 255, 255);
                continue;
            }
            ledSubsystem.setRGB(i, 20, 120, 255);
        }
        ledSubsystem.sendData();
        stage++;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime >= waitTime) {
            for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
                if (i % 4 == stage) {
                    ledSubsystem.setRGB(i, 255, 255, 255);
                    continue;
                }
                ledSubsystem.setRGB(i, 20, 120, 255);
            }
            ledSubsystem.sendData();
            stage = stage + 1 > 3 ? 0 : stage + 1;
            startTime = System.currentTimeMillis();
        }
    }

    public boolean runsWhenDisabled() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}