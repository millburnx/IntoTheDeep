package org.firstinspires.ftc.teamcode.common.subsystems.intake

import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class Intake(val robot: Robot) : Subsystem() {
    val linkage: Linkage = Linkage(robot)
    val subsystems: List<Subsystem> = listOf(linkage)

    override fun init() {
        subsystems.forEach { it.init() }
    }
}