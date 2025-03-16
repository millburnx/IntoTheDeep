package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d

@Autonomous(name = "Sample Auton Faster", preselectTeleOp = "Main Teleop Red")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class FasterSampleAuton2 : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    val currentCommands = mutableListOf<String>()

    fun namedCommand(
        name: String,
        command: Command,
    ) = command.beforeStarting { currentCommands.add(name) }.whenFinished { currentCommands.remove(name) }

    fun delayedSequential(
        delay: Long,
        vararg commands: Command,
    ) = SequentialCommandGroup(
        *commands
            .map {
                listOf(
                    WaitCommand(delay),
                    it,
                )
            }.flatten()
            .toTypedArray(),
    )

    override fun initialize() {
        with(robot) {
            FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)

            robot.drive.pinPoint.reset()
            sleep(500)
            drive.pinPoint.update()
            drive.pose = Pose2d(startingPose)
            drive.pinPoint.update()
            robot.outtake.arm.periodic()
            robot.outtake.claw.close()
            robot.outtake.claw.periodic()
            robot.intake.arm.periodic()
            robot.intake.claw.periodic()
            robot.intake.diffy.resetDiffyRotations()
            robot.doYellow = true
            robot.intake.diffy.resetDiffyRotations()
            doYellow = true

            fun deposit() =
                namedCommand(
                    "deposit",
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            macros.transfer().andThen(
                                outtake.slides.goTo(Slides.State.HIGH_BASKET),
                            ),
                            drive.pid(Pose2d(basketPose)),
                        ),
                        WaitCommand(dropDuration).andThen(
                            outtake.open(),
                        ),
                    ),
                )

            fun pickup(
                count: Int,
                pose: Array<Double>,
            ) = namedCommand(
                "pickup $count",
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        drive.pid(Pose2d(pose)),
                        outtake.base(),
                        intake.extend().andThen(
                            intake.grab(),
                        ),
                    ),
                ),
            )

            schedule(
                delayedSequential(
                    1000,
                    deposit(),
                    pickup(1, sample1Pose),
                    deposit(),
                    pickup(2, sample2Pose),
                    deposit(),
                    pickup(3, sample3Pose),
                    deposit(),
                ),
            )
        }
    }

    override fun exec() {
    }

    companion object {
        @JvmField
        var startingPose = arrayOf(-60.0, 40.0, 0.0)

        @JvmField
        var basketPose = arrayOf(-57.0, 59.0, -45.0)

        @JvmField
        var dropDuration: Long = 500

        @JvmField
        var sample1Pose = arrayOf(-47.0, 49.5, 0.0)

        @JvmField
        var sample2Pose = arrayOf(-47.0, 59.5, 0.0)

        @JvmField
        var sample3Pose = arrayOf(-42.5, 52.5, 45.0)
    }
}
