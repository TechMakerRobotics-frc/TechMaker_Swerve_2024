package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.AlignCommand;
import frc.robot.subsystems.drive.Drive;

public class RegisterAlign {

    /**
     * 
     * Register the alignment in the pathplanner.
     * @param to time out
     * @param drive the drive subsystem
     */
    public RegisterAlign(double to, Drive drive){
        NamedCommands.registerCommand("Align to target 1", new AlignCommand(1, to, drive));
        NamedCommands.registerCommand("Align to target 2", new AlignCommand(2, to, drive));
        NamedCommands.registerCommand("Align to target 3", new AlignCommand(3, to, drive));
        NamedCommands.registerCommand("Align to target 4", new AlignCommand(4, to, drive));
        NamedCommands.registerCommand("Align to target 5", new AlignCommand(5, to, drive));
        NamedCommands.registerCommand("Align to target 6", new AlignCommand(6, to, drive));
        NamedCommands.registerCommand("Align to target 7", new AlignCommand(7, to, drive));
        NamedCommands.registerCommand("Align to target 8", new AlignCommand(8, to, drive));
        NamedCommands.registerCommand("Align to target 9", new AlignCommand(9, to, drive));
        NamedCommands.registerCommand("Align to target 10", new AlignCommand(10, to, drive));
        NamedCommands.registerCommand("Align to target 11", new AlignCommand(11, to, drive));
        NamedCommands.registerCommand("Align to target 12", new AlignCommand(12, to, drive));
        NamedCommands.registerCommand("Align to target 13", new AlignCommand(13, to, drive));
        NamedCommands.registerCommand("Align to target 14", new AlignCommand(14, to, drive));
        NamedCommands.registerCommand("Align to target 15", new AlignCommand(15, to, drive));
        NamedCommands.registerCommand("Align to target 16", new AlignCommand(16, to, drive));
    }
    
}
