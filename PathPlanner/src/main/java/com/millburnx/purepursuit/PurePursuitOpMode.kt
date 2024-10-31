package com.millburnx.purepursuit

import com.acmerobotics.dashboard.canvas.Canvas
import com.millburnx.dashboard.TelemetryPacket
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import kotlin.math.cos
import kotlin.math.sin

class PurePursuitOpMode(ppi: Double, updateHertz: Double = -1.0) : OpMode(ppi, updateHertz) {
    val background = Utils.Colors.bg1
    val colors = listOf(Utils.Colors.red, Utils.Colors.blue, Utils.Colors.green, Utils.Colors.yellow)
    val lookahead = 8.0..16.0
    var purePursuit: PurePursuit = PurePursuit(
        listOf(
            Vec2d(0.0, 0.0),
            Vec2d(48.0, 0.0),
            Vec2d(48.0, -48.0),
            Vec2d(0.0, -48.0),
        ), lookahead
    )

    init {
        ftcDashboard.panel.addMouseListener(object : MouseAdapter() {
            override fun mouseClicked(e: MouseEvent) {
                if (e.button == MouseEvent.BUTTON1) {
                    val x = e.x / ppi - 144.0 / 2
                    val y = e.y / ppi - 144.0 / 2
                    robot.position = Vec2d(x, y)
                } else if (e.button == MouseEvent.BUTTON3) {
                    val x = e.x / ppi - 144.0 / 2
                    val y = e.y / ppi - 144.0 / 2
                    val angle = robot.position.angleTo(Vec2d(x, y))
                    robot.heading = angle
                }
            }
        })
    }

    override fun init() {
        println("Initializing Pure Pursuit")
        ftcDashboard.reset = {
            stop()
            purePursuit = PurePursuit(purePursuit.path, purePursuit.lookahead, purePursuit.threshold)
            prevPositions.clear()
            robot.position = purePursuit.path[0]
            robot.heading = 0.0
            render()
        }
        ftcDashboard.load = {
            stop()
            loadPath()
            ftcDashboard.reset()
        }
    }

    private fun loadPath() {
        val file = Utils.fileDialog("paths", "*.tsv") ?: return
        val pathList = Vec2d.loadList(file)
        purePursuit = PurePursuit(pathList, lookahead)
        render()
    }

    override fun loop(): Boolean {
        val results = purePursuit.calc(robot.position, robot.heading, deltaTime)

        val targetPoint = results.target
        val powerF = robot.position.distanceTo(targetPoint)
        val angleDiff = Util.getAngleDiff((robot.position to robot.heading), targetPoint)
        val powerH = Math.toDegrees(angleDiff)

        robot.drive(powerF, 0.0, powerH)

        render()

        return !results.isFinished
    }

    val prevPositions: MutableList<Vec2d> = mutableListOf()

    fun render() {
        val packet = TelemetryPacket()
        renderPath(packet.fieldOverlay())
        drawRobot(packet.fieldOverlay())
        ftcDashboard.sendTelemetryPacket(packet)
    }

    private fun renderPath(canvas: Canvas) {
//        canvas.setFill(background).fillRect(-144.0 / 2, -144.0 / 2, 144.0, 144.0).setStroke(Utils.Colors.bg2)
        canvas.drawImage("bg.png", 0.0, 0.0, 144.0, 144.0)
//            .drawGrid(0.0, 0.0, 144.0, 144.0, 7, 7)
            .setStrokeWidth(2)

        var color = 0
        for (bezier in purePursuit.beziers) {
            canvas.setStroke(colors[color % colors.size])
            bezier.draw(canvas)
            color++
        }

        color = 0
        val anchorSize = 1.5
        val handleSize = 1.5
        val path = purePursuit.path
        for (i in 0..path.size - 2 step 3) {
            val p0 = path[i].toRR()
            val p1 = path[i + 1].toRR()
            val p2 = path[i + 2].toRR()
            val p3 = path[i + 3].toRR()

            canvas.setStroke(colors[color % colors.size]).strokeLine(p0.x, p0.y, p1.x, p1.y)
                .strokeLine(p2.x, p2.y, p3.x, p3.y).setFill(background).fillCircle(p0.x, p0.y, anchorSize)
                .fillCircle(p1.x, p1.y, handleSize).fillCircle(p2.x, p2.y, handleSize)
                .fillCircle(p3.x, p3.y, anchorSize).setStroke("#FFFFFF").strokeCircle(p0.x, p0.y, anchorSize)
                .strokeCircle(p3.x, p3.y, anchorSize).setStroke(colors[color % colors.size])
                .strokeCircle(p1.x, p1.y, handleSize).strokeCircle(p2.x, p2.y, handleSize)

            color++
        }
    }

    private fun drawRobot(canvas: Canvas) {
        prevPositions.add(robot.position)
        val copyPositions = prevPositions.toList().map { it.toRR() }

        val lookaheadVector = Vec2d(purePursuit.currentLookahead, 0.0).rotate(robot.heading)
        val lookaheadPoint = robot.position + lookaheadVector
        val lookaheadPointRR = lookaheadPoint.toRR()

        val robotPos = robot.position.toRR()

        val robotTopLeft = Vec2d(
            cos(robot.heading) * (-robot.size.x / 2) - sin(robot.heading) * (-robot.size.y / 2),
            sin(robot.heading) * (-robot.size.x / 2) + cos(robot.heading) * (-robot.size.y / 2)
        )
        val robotTopRight = Vec2d(
            cos(robot.heading) * (robot.size.x / 2) - sin(robot.heading) * (-robot.size.y / 2),
            sin(robot.heading) * (robot.size.x / 2) + cos(robot.heading) * (-robot.size.y / 2)
        )
        val robotBottomLeft = Vec2d(
            cos(robot.heading) * (-robot.size.x / 2) - sin(robot.heading) * (robot.size.y / 2),
            sin(robot.heading) * (-robot.size.x / 2) + cos(robot.heading) * (robot.size.y / 2)
        )
        val robotBottomRight = Vec2d(
            cos(robot.heading) * (robot.size.x / 2) - sin(robot.heading) * (robot.size.y / 2),
            sin(robot.heading) * (robot.size.x / 2) + cos(robot.heading) * (robot.size.y / 2)
        )

        val robotCorners =
            listOf(robotTopLeft, robotTopRight, robotBottomRight, robotBottomLeft).map { it.toRR() + robotPos }

        val robotPolyline = listOf(*robotCorners.toTypedArray(), robotCorners[0])

        canvas.setFill("#FFFFFF")
            .strokePolyline(copyPositions.map { it.x }.toDoubleArray(), copyPositions.map { it.y }.toDoubleArray()
            ).setFill(Utils.Colors.purple).fillCircle(robotPos.x, robotPos.y, 1.0)
            .strokeCircle(robotPos.x, robotPos.y, purePursuit.currentLookahead)
            .strokeLine(robotPos.x, robotPos.y, lookaheadPointRR.x, lookaheadPointRR.y)
            .strokePolyline(robotPolyline.map { it.x }.toDoubleArray(), robotPolyline.map { it.y }.toDoubleArray())
    }
}

object Util {
    fun getAngleDiff(a: Pair<Vec2d, Double>, b: Vec2d): Double {
        val (aPoint, aAngle) = a
        val diff = Utils.normalizeAngle(aPoint.angleTo(b))
        return Utils.normalizeAngle(diff - aAngle)
    }
}