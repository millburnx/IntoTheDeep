package org.firstinspires.ftc.teamcode.common

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.utils.conditionalCommand
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.outtakeFlipDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.outtakeLiftingDuration
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.transferClawDelay

class Macros(
    val robot: Robot,
) {
    fun miniTransfer() =
        SequentialCommandGroup(
            ParallelCommandGroup(
                robot.outtake.open(),
                robot.outtake.base(),
                robot.intake.retract(),
            ),
            robot.intake.claw.loose(),
            robot.outtake.transfer(),
            WaitCommand(transferClawDelay),
            robot.outtake.close(),
            WaitCommand(transferClawDelay),
        )

    fun exitTransfer() =
        SequentialCommandGroup(
            robot.intake.open(),
            robot.intake.arm.extended(),
            WaitCommand(outtakeFlipDelay),
            robot.outtake.basketPartial(),
        )

    fun transfer() =
        SequentialCommandGroup(
            miniTransfer(),
            exitTransfer(),
        )

    fun exitSpecPickup() =
        conditionalCommand(
            SequentialCommandGroup(
                robot.outtake.basePartial(),
                WaitCommand(outtakeLiftingDuration),
            ),
        ) { robot.intake.arm.state == IntakeArmPosition.SPECIMEN }
}
