package org.firstinspires.ftc.teamcode.common.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.AxonCR

@Config
class Diffy(
    val robot: Robot,
) : Subsystem() {
    enum class State {
        TRANSFER,
        HOVER,
        PICKUP,
        SWEEP,
        MANUAL,
        DIRECT,
        AUTON_SPECIMEN,
    }

    val left = AxonCR(robot.hardware, "diffyLeft", "analog2", false)
    val right = AxonCR(robot.hardware, "diffyRight", "analog1")

    var state: State = State.TRANSFER

    var pitch: Double = transferPitch // up and down, ideally -1 to 1
    var roll: Double = transferRoll // rotate, ideally -1 to 1

    fun restoreDiffyRotations() {
        println("!!!!LAST $lastLeftRotations, $lastRightRotations")
        setDiffyRotations(lastLeftRotations, lastRightRotations)
    }

    fun resetDiffyRotations() {
        setDiffyRotations(0, 0)
    }

    fun setDiffyRotations(
        left: Int,
        right: Int,
    ) {
        this.left.rotations = left
        lastLeftRotations = left
        this.right.rotations = right
        lastRightRotations = right
    }

    override fun init() {
        restoreDiffyRotations()
    }

    // run in a loo during init
    fun initLoopable() {
        periodic()
    }

    val pidLeft = PIDController(kP, kI, kD)
    val pidRight = PIDController(kP, kI, kD)

    override fun periodic() {
        if (state == State.DIRECT) return // direct control

        pidLeft.setPID(kP, kI, kD)
        pidRight.setPID(kP, kI, kD)

        val (pitch, roll) =
            when (state) {
                State.TRANSFER -> transferPitch to transferRoll
                State.HOVER -> hoverPitch to hoverRoll // use pickup for when you're autorotating
                State.PICKUP -> pickupPitch to this.roll
                State.SWEEP -> sweepPitch to sweepRoll
                State.AUTON_SPECIMEN -> autonSpecimenPitch to autonSpecimenRoll
                else -> return
            }

        // make sure relative offsets work
        this.pitch = pitch
        this.roll = roll

        val leftTarget = pitch - roll
        val rightTarget = pitch + roll
        left.power = pidLeft.calculate(left.position, leftTarget)
        right.power = pidLeft.calculate(right.position, rightTarget)

        lastLeftRotations = left.rotations
        lastRightRotations = right.rotations
    }

    fun transfer() = InstantCommand({ state = State.TRANSFER })

    fun hover() = InstantCommand({ state = State.HOVER })

    fun pickup() = InstantCommand({ state = State.PICKUP })

    fun sweep() = InstantCommand({ state = State.SWEEP })

    fun autonSpecimen() = InstantCommand({ state = State.AUTON_SPECIMEN })

    fun manual() = InstantCommand({ state = State.MANUAL })

    companion object {
        @JvmField
        var kP = -1.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var transferPitch = -.25

        @JvmField
        var transferRoll = .275

        @JvmField
        var hoverPitch = -.65

        @JvmField
        var hoverRoll = -.3

        @JvmField
        var pickupPitch = -.6

        @JvmField
        var sweepPitch = transferPitch

        @JvmField
        var sweepRoll = transferRoll

        @JvmField
        var autonSpecimenPitch = -.2

        @JvmField
        var autonSpecimenRoll = transferRoll

        var lastLeftRotations = 0
        var lastRightRotations = 0
    }
}
