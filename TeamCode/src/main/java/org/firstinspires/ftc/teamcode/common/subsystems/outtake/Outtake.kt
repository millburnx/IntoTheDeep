package org.firstinspires.ftc.teamcode.common.subsystems.outtake

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.outtakeFlipDelay

class Outtake(
    val robot: Robot,
) : Subsystem() {
    val slides: Slides = Slides(robot)
    val arm: OuttakeArm = OuttakeArm(robot)
    val wrist: OuttakeWrist = OuttakeWrist(robot)
    val claw: OuttakeClaw = OuttakeClaw(robot)
    val subsystems: List<Subsystem> = listOf(slides, arm, wrist, claw)

    override fun init() {
        subsystems.forEach { it.init() }
    }

    fun base() =
        ParallelCommandGroup(
            slides.goTo(Slides.min),
            basePartial(),
            WaitCommand(outtakeFlipDelay),
        )

    fun basePartial() =
        ParallelCommandGroup(
            arm.base(),
            wrist.base(),
        )

    fun transfer() =
        ParallelCommandGroup(
            slides.goTo(Slides.min),
            arm.transfer(),
            wrist.transfer(),
        )

    fun basketPartial() =
        ParallelCommandGroup(
            arm.basket(),
            wrist.basket(),
        )

    fun specimenPartial() =
        ParallelCommandGroup(
            arm.specimen(),
            wrist.specimen(),
        )

    fun autonSpecimenPartial() =
        ParallelCommandGroup(
            arm.autonSpecimen(),
            wrist.autonSpecimen(),
        )

    fun specimenPickupPartial() =
        ParallelCommandGroup(
            arm.pickup(),
            wrist.pickup(),
        )

    fun park() =
        ParallelCommandGroup(
            slides.goTo(Slides.min),
            parkPartial(),
        )

    fun parkPartial() =
        ParallelCommandGroup(
            arm.park(),
            wrist.park(),
        )

    fun open() = InstantCommand(claw::open, claw)

    fun close() = InstantCommand(claw::close, claw)
}
