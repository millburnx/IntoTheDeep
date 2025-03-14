package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.outtakeDropArmDelay

@Autonomous(name = "Sample Auton", preselectTeleOp = "Main Teleop Red")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SampleAuton : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    override fun initialize() {
        super.initialize()

        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)

        sleep(500)
        robot.drive.pinPoint.update()

        robot.drive.pose = Pose2d(startingX, startingY, startingHeading)
        robot.drive.pinPoint.update()
        val commands = mutableListOf<Command>()
        robot.outtake.arm.periodic()
        robot.outtake.claw.close()
        robot.outtake.claw.periodic()
        robot.intake.arm.periodic()
        robot.intake.claw.periodic()

        val grab = {
            SequentialCommandGroup(
                robot.autoPickup.startScanning(),
                WaitUntilCommand({ robot.autoPickup.lastTarget != null }),
                robot.autoPickup.stopScanning(),
                robot.autoPickup.align(),
                robot.intake.grab(),
                robot.autoPickup.stop(),
                InstantCommand({ robot.autoPickup.lastTarget = null }),
                robot.macros.miniTransfer(),
            )
        }

        val up = {
            ParallelCommandGroup(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        robot.outtake.arm.basket(),
                        robot.outtake.wrist.basket(),
                    ),
                ),
                SlidesCommand(robot.outtake.slides, Slides.State.HIGH_BASKET),
            )
        }

        val down = {
            SequentialCommandGroup(
                ParallelCommandGroup(
                    InstantCommand({
                        robot.outtake.arm.state = OuttakeArmPosition.BASE
                        robot.outtake.wrist.state = OuttakeWristPosition.BASE
                    }, robot.outtake.arm, robot.outtake.wrist),
                    WaitCommand(outtakeDropArmDelay),
                ),
                SlidesCommand(robot.outtake.slides, Slides.State.BASE),
            )
        }

        val drop = {
            SequentialCommandGroup(
                robot.outtake.open(),
                WaitCommand(250),
                ParallelCommandGroup(
                    ParallelCommandGroup(
                        robot.outtake.arm.base(),
                        robot.outtake.wrist.base(),
                        WaitCommand(outtakeDropArmDelay),
                    ),
                    robot.drive.pid(Pose2d(basketX, basketY, -45.0)),
                ),
            )
        }

        val basket = {
            SequentialCommandGroup(
                ParallelCommandGroup(
                    SequentialCommandGroup(
                        WaitUntilCommand({ robot.outtake.slides.position > Slides.min + (Slides.highBasket - Slides.min) / 2 }),
                        robot.drive.pid(Pose2d(basketX, basketY, -45.0)),
                    ),
                    up(),
                ),
                WaitCommand(basketDuration),
                drop(),
            )
        }

        commands.add(
            SequentialCommandGroup(
                basket(),
                ParallelCommandGroup(
                    down(),
                    robot.drive.pid(Pose2d(sample1X, sample1Y, 0.0)),
                    WaitCommand(pidStablize),
                    robot.intake.extend(),
                ),
                WaitCommand(grabDuration),
                grab(),
                basket(),
                ParallelCommandGroup(
                    down(),
                    robot.drive.pid(Pose2d(sample2X, sample2Y, 0.0)),
                    WaitCommand(pidStablize),
                    robot.intake.extend(),
                ),
                WaitCommand(grabDuration),
                grab(),
                basket(),
                ParallelCommandGroup(
                    down(),
                    robot.drive.pid(Pose2d(sample3X, sample3Y, sample3H)),
                    WaitCommand(pidStablize),
                    robot.intake.extend(),
                ),
                WaitCommand(grabDuration),
                grab(),
                basket(),
                ParallelCommandGroup(
                    SlidesCommand(robot.outtake.slides, Slides.State.BASE),
                    robot.outtake.arm.park(),
                    robot.outtake.wrist.park(),
                    robot.drive.purePursuit("parkSamples", 90.0, true),
                ),
            ),
        )

        schedule(SequentialCommandGroup(*commands.toTypedArray()))

        while (!isStarted()) {
            robot.intake.diffy.initLoopable()
        }
    }

    companion object {
        @JvmField
        var startingX = -60.0

        @JvmField
        var startingY = 40.0

        @JvmField
        var startingHeading = 0.0

        @JvmField
        var sample1X = -47.0

        @JvmField
        var sample1Y = 49.5

        @JvmField
        var sample2X = -47.0

        @JvmField
        var sample2Y = 59.5

        @JvmField
        var sample3X = -42.5

        @JvmField
        var sample3Y = 52.5

        @JvmField
        var sample3H = 45.0

        @JvmField
        var basketX = -57.0

        @JvmField
        var basketY = 59.0

        @JvmField
        var basketDuration: Long = 500

        @JvmField
        var grabDuration: Long = 250

        @JvmField
        var pidStablize: Long = 500
    }

    override fun exec() {
    }
}
