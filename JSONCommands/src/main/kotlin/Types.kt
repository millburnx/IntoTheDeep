package com.millburnx.jsoncommands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.ParallelRaceGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.SubsystemBase

public interface BaseObject {
    public val type: String

    public fun generate(): Any
}

public interface Subsystem : BaseObject {
    override fun generate(): SubsystemBase
}

public interface Command : BaseObject {
    override fun toString(): String

    override fun generate(): CommandBase
}

public class EmptyCommand : Command {
    override val type: String = "Command/EmptyCommand"

    override fun generate(): CommandBase = object : CommandBase() {}

    override fun toString(): String = "EmptyCommand()"
}

public interface CommandGroup : Command {
    public val commands: List<Command>

    override fun generate(): com.arcrobotics.ftclib.command.CommandGroupBase
}

public class SequentialCommandGroup : CommandGroup {
    override val type: String = "CommandGroup/SequentialCommandGroup"

    override fun generate(): SequentialCommandGroup = SequentialCommandGroup()

    override val commands: List<Command> = emptyList()

    override fun toString(): String = "SequentialCommandGroup(commands=$commands)"
}

public class ParallelCommandGroup : CommandGroup {
    override val type: String = "CommandGroup/ParallelCommandGroup"

    override fun generate(): ParallelCommandGroup = ParallelCommandGroup(*commands.map { it.generate() }.toTypedArray())

    override val commands: List<Command> = emptyList()

    override fun toString(): String = "ParallelCommandGroup(commands=$commands)"
}

public class ParallelRaceGroup : CommandGroup {
    override val type: String = "CommandGroup/ParallelRaceGroup"

    override fun generate(): ParallelRaceGroup = ParallelRaceGroup(*commands.map { it.generate() }.toTypedArray())

    override val commands: List<Command> = emptyList()

    override fun toString(): String = "ParallelRaceGroup(commands=$commands)"
}

public class ParallelDeadlineGroup : CommandGroup {
    override val type: String = "CommandGroup/ParallelDeadlineGroup"

    override fun generate(): ParallelDeadlineGroup =
        ParallelDeadlineGroup(deadline.generate(), *commands.map { it.generate() }.toTypedArray())

    override val commands: List<Command> = emptyList()

    public val deadline: Command = EmptyCommand()

    override fun toString(): String = "ParallelDeadlineGroup(commands=$commands)"
}
