package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import com.millburnx.utils.vec2d.Vec2dInt
import java.awt.Graphics2D

class Circle(
    val x: Double,
    val y: Double,
    val radius: Double,
    val stroke: Boolean,
) : CanvasOpImpl(Type.CIRCLE) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        setColor(ctx, stroke)
        val (topLeft, diameter) = getDrawData(scale)
        if (stroke) {
            return drawStroke(ctx.g2d, topLeft, diameter)
        }
        return drawFill(ctx.g2d, topLeft, diameter)
    }

    private fun drawFill(g2d: Graphics2D, topLeft: Vec2dInt, diameter: Int) {
        g2d.fillOval(topLeft.x, topLeft.y, diameter, diameter)
    }

    private fun drawStroke(g2d: Graphics2D, topLeft: Vec2dInt, diameter: Int) {
        g2d.drawOval(topLeft.x, topLeft.y, diameter, diameter)
    }

    private fun getDrawData(scale: Scale): Pair<Vec2dInt, Int> {
        val center = Vec2dInt(x, y)
        val topLeft = scale.ppiScale(center - radius)
        val diameter = scale.ppiScale(radius * 2)
        return (topLeft to diameter)
    }
}