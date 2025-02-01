package com.millburnx.jsoncommands

public interface BaseObject {
    public val type: String
}

public class Subsystem : BaseObject {
    override val type: String = "Subsystem"
    public var id: String = ""

    override fun toString(): String = "Subsystem(id=$id)"
}

public interface Command : BaseObject {
    override fun toString(): String
}

public interface CommandGroup : Command {
    public val commands: List<Command>
    public val subsystems: List<Subsystem>?
}

public class SequentialCommandGroup : CommandGroup {
    override val type: String = "CommandGroup/SequentialCommandGroup"
    override val commands: List<Command> = emptyList()
    override val subsystems: List<Subsystem>? = null

    override fun toString(): String = "SequentialCommandGroup(commands=$commands, subsystems=$subsystems)"
}

public class ParallelCommandGroup : CommandGroup {
    override val type: String = "CommandGroup/ParallelCommandGroup"
    override val commands: List<Command> = emptyList()
    override val subsystems: List<Subsystem>? = null

    override fun toString(): String = "ParallelCommandGroup(commands=$commands, subsystems=$subsystems)"
}

public class ParallelRaceGroup : CommandGroup {
    override val type: String = "CommandGroup/ParallelRaceGroup"
    override val commands: List<Command> = emptyList()
    override val subsystems: List<Subsystem>? = null

    override fun toString(): String = "ParallelRaceGroup(commands=$commands, subsystems=$subsystems)"
}

public class ParallelDeadlineGroup : CommandGroup {
    override val type: String = "CommandGroup/ParallelDeadlineGroup"
    override val commands: List<Command> = emptyList()
    override val subsystems: List<Subsystem>? = null

    override fun toString(): String = "ParallelDeadlineGroup(commands=$commands, subsystems=$subsystems)"
}
