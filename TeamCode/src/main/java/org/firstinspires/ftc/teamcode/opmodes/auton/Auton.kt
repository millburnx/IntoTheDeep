package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Path
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.commands.PIDCommand
import org.firstinspires.ftc.teamcode.common.commands.PickupGroup
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.commands.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.ReturnToBase
import org.firstinspires.ftc.teamcode.common.commands.SpecimenScore
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.FasterSampleDetection
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.tuning.PurePursuitTest
import java.io.File

@Config
object AutonConfig2 {
    @JvmField
    var startX = -60.0

    @JvmField
    var startY = -12.0

    @JvmField
    var startH = 0.0

    @JvmField
    var specimenPower: Double = 0.4

    @JvmField
    var specimenDuration: Long = 875

    @JvmField
    var pickupX: Double = -55.0

    @JvmField
    var pickupY: Double = -44.0

    @JvmField
    var offsetMulti: Double = 1.0

    @JvmField
    var humanDuration: Long = 2000

    @JvmField
    var humanOffset: Double = 12.0
}

@Autonomous(name = "Full Auton")
class BlueAuton : CommandOpMode() {
    val drive: Drive by lazy {
        Drive(
            hardwareMap,
            tel,
            dash,
            Vec2d(AutonConfig2.startX, AutonConfig2.startY),
            AutonConfig2.startH,
            zeroBreak = true
        )
    }
    val arm: Arm by lazy {
        Arm(hardwareMap, telemetry, lift.lift::getCurrentPosition)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }
    val intake: Intake by lazy {
        Intake(hardwareMap)
    }

    val parkServo: Servo by lazy {
        hardwareMap["parkServo"] as Servo
    }
    val tel: Telemetry = Telemetry()
    val dash: FtcDashboard = FtcDashboard.getInstance()
    val paths = mutableListOf<Path>()

    val rootDir = Environment.getExternalStorageDirectory()

    fun loadSegment(index: Int): Path {
        return loadPath("${AutonConfig.pathName}${index}")
    }

    fun loadPath(name: String): Path {
        return PurePursuitTest.Companion.pathMap.getOrPut(name) {
            val path = "${rootDir}/Paths/${name}.tsv"
            val segment: Path = try {
                val loaded = Vec2d.loadList(File(path))
                println(loaded)
                loaded
            } catch (e: Error) {
                e.printStackTrace()
                println("PATH (`$path`) NOT FOUND")
                Path(listOf())
            }
            return segment
        }
    }

    companion object {
        val pathMap: MutableMap<Int, Path> = mutableMapOf()
    }

    val pidX = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidY = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidH = APIDController(AutonConfig.pidH_kP, AutonConfig.pidH_kI, AutonConfig.pidH_kD)


    val samplePipeline: FasterSampleDetection by lazy { FasterSampleDetection(telemetry) }
    val visionPortal: VisionPortal by lazy {
        VisionPortal(
            hardwareMap,
            "camera1",
            listOf(samplePipeline)
        )
    }

    fun purePursuitSegment(segment: Path) = PurePursuitCommand(
        drive,
        segment.points,
        segment.endHeading,
        dash,
        pidX,
        pidY,
        pidH,
        AutonConfig.minRange..AutonConfig.maxRange
    ).withTimeout(AutonConfig.timeout)

    fun pidSegment(
        endPoint: Vec2d,
        endHeading: Double? = null,
        multiF: Double = 1.0,
        multiH: Double = 1.0,
        threshold: Double = AutonConfig.threshold,
        thresholdHeading: Double = AutonConfig.headingTolerance,
    ): PIDCommand =
        PIDCommand(
            drive,
            endPoint,
            endHeading,
            pidX,
            pidY,
            pidH,
            multiF,
            multiH,
            threshold,
            thresholdHeading
        )


    override fun initialize() {
        //start at (-60, 12)
        lift.armAngle = arm::angle
        intake.close()
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)

        val commands: MutableList<Command> = mutableListOf()

        val score = fun(offset: Int): SequentialCommandGroup =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    pidSegment(Vec2d(-46, -10 + offset * AutonConfig2.offsetMulti), 0.0),
                    SpecimenScore(arm, lift, intake)
                ),
                RelativeDrive(
                    drive,
                    AutonConfig2.specimenPower
                ).withTimeout(AutonConfig2.specimenDuration),
                InstantCommand(intake::open),
                ParallelCommandGroup(
                    pidSegment(Vec2d(-44, -10 + offset * AutonConfig2.offsetMulti), 0.0),
                    ReturnToBase(arm, lift)
                )
            )
        val pickup = SequentialCommandGroup(
            pidSegment(
                Vec2d(AutonConfig2.pickupX, AutonConfig2.pickupY),
                -180.0,
                threshold = AutonConfig.threshold * 4
            ),
            PickupGroup(drive, arm, lift, intake, visionPortal.cameraSize, samplePipeline.detections::get, true),
        )
        val push1 = SequentialCommandGroup(
            purePursuitSegment(loadPath("push1")),
            pidSegment(
                Vec2d(-12, -45),
                0.0,
                threshold = AutonConfig.threshold * 2
            ),
            pidSegment(
                Vec2d(-52, -45),
                0.0,
                threshold = AutonConfig.threshold * 2
            ),
        )
        val push2 = SequentialCommandGroup(
            pidSegment(
                Vec2d(-12, -38),
                0.0,
                threshold = AutonConfig.threshold * 4
            ),
            pidSegment(
                Vec2d(-12, -53),
                0.0,
                threshold = AutonConfig.threshold * 2
            ),
            pidSegment(
                Vec2d(-56, -56),
                0.0,
                threshold = AutonConfig.threshold * 4
            ),
            pidSegment(
                Vec2d(-40, -56),
                0.0,
                threshold = AutonConfig.threshold * 4
            ),
            pidSegment(
                Vec2d(-40, AutonConfig2.pickupY),
                -180.0,
                threshold = AutonConfig.threshold * 4
            ),
            WaitCommand(AutonConfig2.humanDuration),
        )

        commands.add(ReturnToBase(arm, lift))
        commands.add(score(0))
        commands.add(push1)
        commands.add(push2)
        commands.add(pickup)
        commands.add(score(1))
        commands.add(
            pidSegment(
                Vec2d(AutonConfig2.pickupX, AutonConfig2.pickupY - 4.0),
                90.0,
                threshold = AutonConfig.threshold * 2
            ),
        )
        schedule(
            SequentialCommandGroup(*commands.toTypedArray())
        )
    }

    override fun run() {
        super.run()
    }
}