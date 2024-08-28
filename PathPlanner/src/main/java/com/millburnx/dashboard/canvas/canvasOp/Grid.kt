package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import com.millburnx.utils.vec2d.Vec2d
import com.millburnx.utils.vec2d.Vec2dInt
import java.awt.Graphics2D

class Grid(
    val x: Double,
    val y: Double,
    private val width: Double,
    private val height: Double,
    private val numTicksX: Int,
    private val numTicksY: Int,
    private val theta: Double,
    private val pivotX: Double,
    private val pivotY: Double,
    private val usePageFrame: Boolean,
) : CanvasOpImpl(Type.GRID) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        val g2d = setup(ctx, scale)
        val topLeft = scale.ppiScale(Vec2d(x to y))
        val size = scale.ppiScale(Vec2d(width to height))
        val steps = getSteps(scale)
        drawGrid(g2d, topLeft, size, steps)
        g2d.dispose()
    }

    fun setup(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale): Graphics2D {
        val g2d = ctx.g2d.create() as Graphics2D
        val transform = CanvasTransform(theta, pivotX, pivotY, usePageFrame)
        g2d.transform = getTransform(ctx, scale, transform)
        setColor(ctx, stroke = true)
        return g2d
    }

    fun getSteps(scale: Scale): Vec2dInt {
        val ticks = Vec2d(numTicksX, numTicksY)
        val size = Vec2d(width, height)
        return scale.ppiScale(size / ticks)
    }

    fun drawGrid(g2d: Graphics2D, topLeft: Vec2dInt, size: Vec2dInt, steps: Vec2dInt) {
        for (i in 0..numTicksX) {
            val xCoord = topLeft.x + i * steps.x
            g2d.drawLine(xCoord, topLeft.y, xCoord, topLeft.y + size.y)
        }
        for (i in 0..numTicksY) {
            val yCoord = topLeft.y + i * steps.y
            g2d.drawLine(topLeft.x, yCoord, topLeft.x + size.x, yCoord)
        }
    }
}