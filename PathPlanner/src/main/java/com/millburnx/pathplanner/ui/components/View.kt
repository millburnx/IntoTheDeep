package com.millburnx.pathplanner.ui.components

import com.millburnx.pathplanner.PointManager
import com.millburnx.pathplanner.ui.Theme
import com.millburnx.utils.BezierPoint
import com.millburnx.utils.Scale
import com.millburnx.utils.vec2d.Vec2d
import com.millburnx.utils.vec2d.Vec2dInt
import java.awt.BasicStroke
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.event.ComponentAdapter
import java.awt.event.ComponentEvent
import java.awt.event.MouseAdapter
import javax.swing.JPanel
import javax.swing.border.MatteBorder

class View : JPanel() {
    var scale = Scale(1.0, Theme.scale)
    var fieldWidth = 144.0 // inches

    val pointManager: PointManager = PointManager()

    init {
        border = MatteBorder(1, 1, 1, 1, Theme.borderColor)
        background = Theme.bg1

        addComponentListener(object : ComponentAdapter() {
            override fun componentResized(e: ComponentEvent) {
                val ppi = width / fieldWidth
                scale = Scale(ppi, Theme.scale)
                repaint()
            }
        })

        addMouseListener(object : MouseAdapter() {
            override fun mousePressed(e: java.awt.event.MouseEvent) {
                val viewPos = Vec2d(e.x, e.y)
                val size = Vec2d(size.width, size.height)
                val fieldPos = (viewPos - size / 2) / scale.ppi
                pointManager.addPoint(BezierPoint(fieldPos))
                println("FieldPos: $fieldPos ViewPos: $viewPos Points: ${pointManager.points.size}")
                repaint()
            }
        })
    }


    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val g2d = g.create() as Graphics2D
        setAA(g2d)
        g2d.translate(width / 2, height / 2)

        drawGrid(g2d, Vec2dInt(6, 6))
        drawPoints(g2d)
        g2d.dispose()
    }

    fun drawGrid(g: Graphics2D, tiles: Vec2dInt) {
        val g2d = g.create() as Graphics2D
        g2d.translate(-width / 2, -height / 2)
        g2d.color = Theme.borderColor
        g2d.stroke = BasicStroke(2f)
        for (i in 0 until tiles.x) {
            val x = i * width / tiles.x
            g2d.drawLine(x, 0, x, height)
        }
        for (i in 0 until tiles.y) {
            val y = i * height / tiles.y
            g2d.drawLine(0, y, width, y)
        }
        g2d.dispose()
    }

    fun drawPoints(g: Graphics2D) {
        println("Scale $scale")
        val g2d = g.create() as Graphics2D
        g2d.color = Theme.fg
        g2d.stroke = BasicStroke(4f)
        for (point in pointManager.points) {
            val size = 4.0
            val topLeft = scale.ppiScale(point.anchor - Vec2d(size, size) / 2)
            val sizeScaled = scale.ppiScale(Vec2d(size, size))
            g2d.fillOval(topLeft.x, topLeft.y, sizeScaled.x, sizeScaled.y)
        }
        g2d.color = Theme.primary
        println("Beziers: ${pointManager.beziers.size}, ${pointManager.beziers}")
        for (bezier in pointManager.beziers) {
            val samples = 100
            var lastPoint = bezier.at(0.0)
            for (i in 1 until samples) {
                val t = i.toDouble() / samples
                val point = bezier.at(t)
                g2d.drawLine(
                    scale.ppiScale(lastPoint).x,
                    scale.ppiScale(lastPoint).y,
                    scale.ppiScale(point).x,
                    scale.ppiScale(point).y
                )
                lastPoint = point
            }
        }
        g2d.dispose()
    }

    fun setAA(g2d: Graphics2D) {
        g2d.setRenderingHint(java.awt.RenderingHints.KEY_ANTIALIASING, java.awt.RenderingHints.VALUE_ANTIALIAS_ON)
        g2d.setRenderingHint(
            java.awt.RenderingHints.KEY_TEXT_ANTIALIASING,
            java.awt.RenderingHints.VALUE_TEXT_ANTIALIAS_ON
        )
        g2d.setRenderingHint(
            java.awt.RenderingHints.KEY_FRACTIONALMETRICS,
            java.awt.RenderingHints.VALUE_FRACTIONALMETRICS_ON
        )
        g2d.setRenderingHint(
            java.awt.RenderingHints.KEY_RENDERING,
            java.awt.RenderingHints.VALUE_RENDER_QUALITY
        )
        g2d.setRenderingHint(
            java.awt.RenderingHints.KEY_STROKE_CONTROL,
            java.awt.RenderingHints.VALUE_STROKE_PURE
        )
    }
}