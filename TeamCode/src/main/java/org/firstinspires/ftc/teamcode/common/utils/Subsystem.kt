package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.command.SubsystemBase
import com.millburnx.jsoncommands.Subsystem

abstract class Subsystem : SubsystemBase() {
    open fun init() {
        subsystems.forEach { it.init() }
    }

    open val subsystems: List<org.firstinspires.ftc.teamcode.common.utils.Subsystem> = emptyList()

    fun getJSONSubsystems(): List<JSONSubsystem> {
        val self = JSONSubsystem()
        val children = subsystems.map { it.getJSONSubsystems() }.flatten()
        return listOf(self) + children
    }

    inner class JSONSubsystem : Subsystem {
        override val type: String = "BaseSubsystem"

        override fun generate(): SubsystemBase {
            TODO("Not yet implemented")
        }
    }
}
