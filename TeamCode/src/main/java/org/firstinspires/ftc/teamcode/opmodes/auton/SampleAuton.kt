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
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.outtakeDropArmDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.transferClawDelay

@Autonomous(name = "Sample Auton", preselectTeleOp = "New Teleop")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SampleAuton : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    override fun initialize() {
        super.initialize()

        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)

        robot.drive.pose = Pose2d(startingX, startingY, startingHeading)
        val commands = mutableListOf<Command>()
        robot.outtake.arm.periodic()
        robot.outtake.claw.close()
        robot.outtake.claw.periodic()
        robot.intake.arm.periodic()
        robot.intake.diffy.periodic()
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
                ParallelCommandGroup(
                    robot.intake.retract(),
                    robot.outtake.open(),
                    robot.outtake.base(),
                ),
                robot.outtake.close(),
                WaitCommand(transferClawDelay),
                robot.intake.open(),
            )
        }

        val up = {
            SequentialCommandGroup(
                InstantCommand({
                    robot.outtake.arm.state = OuttakeArmPosition.BASKET
                    robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                }, robot.outtake),
                SlidesCommand(robot.outtake.slides, Slides.highBasket),
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
                SlidesCommand(robot.outtake.slides, Slides.min),
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
//                InstantCommand({ robot.intake.diffy.roll = sample3Roll }),
                WaitCommand(grabDuration),
                grab(),
                basket(),
                ParallelCommandGroup(
                    SlidesCommand(robot.outtake.slides, Slides.min),
                    robot.outtake.arm.park(),
                    robot.outtake.wrist.park(),
                    robot.drive.purePursuit("parkSamples", 90.0),
                ),
            ),
        )

        schedule(SequentialCommandGroup(*commands.toTypedArray()))
    }

    companion object {
        @JvmField
        var startingX = -58.0

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
        var sample3X = -40.5

        @JvmField
        var sample3Y = 52.5

        @JvmField
        var sample3H = 45.0

        @JvmField
        var sample3Roll = -0.5

        @JvmField
        var basketX = -56.0

        @JvmField
        var basketY = 58.0

        @JvmField
        var basketDuration: Long = 500

        @JvmField
        var grabDuration: Long = 250

        @JvmField
        var pidStablize: Long = 250
    }

    override fun exec() {
    }
}
