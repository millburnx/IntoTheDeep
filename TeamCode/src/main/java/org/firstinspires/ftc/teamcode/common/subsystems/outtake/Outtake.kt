package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class Outtake(val robot: Robot) : Subsystem() {
    val slides: Slides = Slides(robot)
    val arm: OuttakeArm = OuttakeArm(robot)
    val wrist: OuttakeWrist = OuttakeWrist(robot)

    //    val claw: OuttakeClaw = OuttakeClaw(robot)
    val subsystems: List<Subsystem> = listOf(slides, arm, wrist)

    override fun init() {
        subsystems.forEach { it.init() }
    }
}