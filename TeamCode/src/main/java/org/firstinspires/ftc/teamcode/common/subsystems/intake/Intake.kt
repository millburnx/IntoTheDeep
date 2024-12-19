package org.firstinspires.ftc.teamcode.common.subsystems.intake

import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class Intake(val robot: Robot) : Subsystem() {
    val linkage: Linkage = Linkage(robot)
    val arm: IntakeArm = IntakeArm(robot)
    val diffy: Diffy = Diffy(robot)
    val claw: IntakeClaw = IntakeClaw(robot)
    val subsystems: List<Subsystem> = listOf(linkage, arm, diffy, claw)

    override fun init() {
        subsystems.forEach { it.init() }
    }
}