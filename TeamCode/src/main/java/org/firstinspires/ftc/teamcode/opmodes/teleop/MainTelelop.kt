package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.commands.DriveRobotCommand
import org.firstinspires.ftc.teamcode.common.commands.PickupGroup
import org.firstinspires.ftc.teamcode.common.commands.ReturnToBase
import org.firstinspires.ftc.teamcode.common.commands.SampleScore
import org.firstinspires.ftc.teamcode.common.commands.SpecimenScore
import org.firstinspires.ftc.teamcode.common.commands.SubmersibleGroup
import org.firstinspires.ftc.teamcode.common.commands.SummersibleEnter
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.misc.DeltaTime
import org.firstinspires.ftc.teamcode.common.subsystems.misc.RisingEdge
import org.firstinspires.ftc.teamcode.common.subsystems.vision.FasterSampleDetection
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal
import org.firstinspires.ftc.teamcode.common.utils.GamepadSRL
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import kotlin.math.abs

@Config
@TeleOp(name = "MainTeleOp")
class MainTelelop : CommandOpMode() {
    val tel: Telemetry = Telemetry()
    val dash: FtcDashboard = FtcDashboard.getInstance()
    val telem = MultipleTelemetry(telemetry, dash.telemetry)
    val samplePipeline: FasterSampleDetection by lazy { FasterSampleDetection(telemetry) }
    val visionPortal: VisionPortal by lazy {
        VisionPortal(
            hardwareMap,
            "camera1",
            listOf(samplePipeline)
        )
    }
    val drive: Drive by lazy {
        Drive(hardwareMap, tel, dash, startingH = startingHeading)
    }
    val deltaTime: DeltaTime = DeltaTime()
    val gp1: GamepadSRL by lazy {
        GamepadSRL(GamepadEx(gamepad1), TeleopBeta.maxLeftRate, TeleopBeta.maxRightRate, deltaTime)
    }
    val gp2: GamepadSRL by lazy {
        GamepadSRL(GamepadEx(gamepad2), TeleopBeta.maxLeftRate, TeleopBeta.maxRightRate, deltaTime)
    }
    val arm: Arm by lazy {
        Arm(hardwareMap, telemetry, lift.lift::getCurrentPosition, startingArm)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap, startingLift)
    }
    val intake: Intake by lazy {
        Intake(hardwareMap)
    }
    var slowDriveMode: Boolean = false;
    var slowMechMode: Boolean = false;

    val triangle = GamepadKeys.Button.Y
    val circle = GamepadKeys.Button.B
    val cross = GamepadKeys.Button.A
    val square = GamepadKeys.Button.X

    val samplePickup by lazy {
        PickupGroup(
            drive, arm, lift, intake,
            visionPortal.cameraSize,
            samplePipeline.detections::get
        )
    }
    val samplePickupTrigger by lazy {
        RisingEdge(gp1, circle) {
            println("sample pickup ${samplePickup.isScheduled} ${samplePickup.isFinished}")
            if (samplePickup.isScheduled) {
                // cancel
                samplePickup.cancel()
                return@RisingEdge
            }
            schedule(samplePickup)
        }
    }

    val specimenPickup by lazy {
        PickupGroup(
            drive, arm, lift, intake,
            visionPortal.cameraSize,
            samplePipeline.detections::get, true
        )
    }
    val specimenPickupTrigger by lazy {
        RisingEdge(gp1, square) {
            if (specimenPickup.isScheduled) {
                // cancel
                specimenPickup.cancel()
                return@RisingEdge
            }
            schedule(specimenPickup)
        }
    }
    val sampleScore by lazy { SampleScore(arm, lift, intake) }
    val sampleScoreTrigger by lazy {
        RisingEdge(gp1, cross) {
            schedule(sampleScore)
        }
    }
    val specimenScore1 by lazy { SpecimenScore(arm, lift, intake) }
    val specimenScoreTrigger by lazy {
        RisingEdge(gp1, triangle) {
            schedule(specimenScore1)
        }
    }
    val submersibleEnter by lazy {
        SummersibleEnter(
            drive,
            arm,
            lift,
            intake,
            visionPortal.cameraSize,
            samplePipeline.detections::get,
            telem
        )
    }
    val submersibleEnterTrigger by lazy {
        RisingEdge(gp1, GamepadKeys.Button.DPAD_UP) {
            schedule(submersibleEnter)
        }
    }
    val submersibleScore by lazy {
        SubmersibleGroup(
            drive,
            arm, // ooga booga
            lift,
            intake,
            visionPortal.cameraSize,
            samplePipeline.detections::get,
            telem
        )
    }
    val submersibleScoreTrigger by lazy {
        RisingEdge(gp1, GamepadKeys.Button.DPAD_DOWN) {
            schedule(submersibleScore)
        }
    }

    val returnToBase by lazy { ReturnToBase(arm, lift) }
    val returnToBaseTrigger by lazy {
        RisingEdge(gp1, GamepadKeys.Button.LEFT_BUMPER) {
            schedule(returnToBase)
        }
    }

    val intakeToggle by lazy {
        InstantCommand({
            val prev = intake.open
            intake.toggle()
            println("intake, $prev -> ${intake.open}")
        })
    }
    val intakeToggleTrigger by lazy {
        RisingEdge(gp1, GamepadKeys.Button.DPAD_LEFT) {
            schedule(intakeToggle)
        }
    }
    val intakeToggle2 by lazy {
        InstantCommand({
            val prev = intake.open
            intake.toggle()
            println("intake, $prev -> ${intake.open}")
        })
    }
    val intakeToggleTrigger2 by lazy {
        RisingEdge(gp2, GamepadKeys.Button.LEFT_BUMPER) {
            schedule(intakeToggle2)
        }
    }
    val resetImu by lazy {
        InstantCommand({
            drive.imu.resetYaw()
            drive.startingH = 0.0
        })
    }
    val resetImuToggle by lazy {
        RisingEdge(gp2, circle) {
            schedule(resetImu)
        }
    }
    val resetRest by lazy {
        InstantCommand({
            arm.resetEncoders()
            lift.resetEncoders()
            lift.bypass = false
        })
    }
    val resetRestToggle by lazy {
        RisingEdge(gp2, cross) {
            schedule(resetRest)
        }
    }
    val armOff by lazy {
        InstantCommand({
            arm.off()
        })
    }
    val armOffToggle by lazy {
        RisingEdge(gp2, GamepadKeys.Button.X) {
            schedule(armOff)
        }
    }
    val liftBypass by lazy {
        InstantCommand({
            lift.bypass = true
        })
    }
    val liftBypassTrigger by lazy {
        RisingEdge(gp2, GamepadKeys.Button.Y) {
            schedule(liftBypass)
        }
    }
    val parkServo: Servo by lazy {
        hardwareMap["parkServo"] as Servo
    }
    var hasStarted: Boolean = false

    override fun initialize() {
        drive.defaultCommand =
            RunCommand({
                schedule(
                    DriveRobotCommand(
                        drive, gp1, telemetry,
                        { slowDriveMode }, { d1CubicAll || !d1Cubic && slowDriveMode }
                    )
                )
            }, drive)
        lift.armAngle = arm::angle
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)

        samplePickupTrigger
        specimenPickupTrigger
        sampleScoreTrigger
        specimenScoreTrigger
        submersibleEnterTrigger
        submersibleScoreTrigger
        returnToBaseTrigger
        intakeToggleTrigger
        intakeToggleTrigger2
        resetImuToggle
        resetRestToggle
        armOffToggle
        liftBypassTrigger

        gp1.gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand({ slowDriveMode = true })
        ).whenReleased(
            InstantCommand({ slowDriveMode = false })
        )
        gp2.gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand({ slowMechMode = true })
        ).whenReleased(
            InstantCommand({ slowMechMode = false })
        )

        gp2.gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whileHeld(
                InstantCommand(
                    { arm.on(); arm.target += if (slowMechMode) slowManualArm else manualArm },
                    arm
                )
            )
        gp2.gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whileHeld(
                InstantCommand(
                    { arm.on(); arm.target -= if (slowMechMode) slowManualArm else manualArm },
                    arm
                )
            )

        schedule(ReturnToBase(arm, lift))
    }

    override fun run() {
        super.run()

        if (!hasStarted) {
            schedule(InstantCommand(intake::open, intake))
            schedule(InstantCommand({ parkServo.position = 1.0 }))
            hasStarted = true
        }

        if (abs(gp2.gamepad.leftX) > 0.1 || abs(gp2.gamepad.leftY) > 0.1 || abs(gp2.gamepad.rightX) > 0.1) {
            schedule(
                DriveRobotCommand(drive, gp2, telemetry, { true }, { !d2Cubic })
            )
        }

        if (gamepad2.left_trigger > 0.1) {
            schedule(
                InstantCommand(
                    { lift.target -= gamepad2.left_trigger * if (slowMechMode) slowManualLift else manualLift },
                    lift
                )
            )
        } else if (gamepad2.right_trigger > 0.1) {
            schedule(
                InstantCommand(
                    { lift.target += gamepad2.right_trigger * if (slowMechMode) slowManualLift else manualLift },
                    lift
                )
            )
        } else if (gamepad1.left_trigger > 0.1) {
            schedule(
                InstantCommand(
                    { lift.target -= gamepad1.left_trigger * if (slowMechMode) slowManualLift else manualLift },
                    lift
                )
            )
        } else if (gamepad1.right_trigger > 0.1) {
            schedule(
                InstantCommand(
                    { lift.target += gamepad1.right_trigger * if (slowMechMode) slowManualLift else manualLift },
                    lift
                )
            )
        }

        telem.addData("arm pos: ", arm.position)
        telem.addData("arm angle: ", arm.angle)
        telem.addData("arm target: ", arm.target)
        telem.addData("arm power: ", -arm.rightRotate.power)

        telem.addData("lift pos: ", lift.position)
        telem.addData("lift target; ", lift.target)
        telem.addData("lift power: ", lift.lift.power)

        telem.addData("intake open: ", intake.open)

        val pose = drive.pose
        telem.addData("x", pose.x)
        telem.addData("y", pose.y)
        telem.addData("heading", Math.toDegrees(pose.heading))

        telem.update()
    }

    companion object {
        @JvmField
        var fieldCentric: Boolean = true

        @JvmField
        var flipY: Boolean = false

        @JvmField
        var manualArm = 4.0

        @JvmField
        var slowManualArm = 1.0

        @JvmField
        var manualLift = 75.0

        @JvmField
        var slowManualLift = 15.0

        @JvmField
        var d1Cubic = true

        @JvmField
        var d1CubicAll = true

        @JvmField
        var d2Cubic = true

        @JvmField
        var startingArm: Double = -20.0

        @JvmField
        var startingLift: Int = 35

        @JvmField
        var startingHeading: Double = 90.0
    }
}
