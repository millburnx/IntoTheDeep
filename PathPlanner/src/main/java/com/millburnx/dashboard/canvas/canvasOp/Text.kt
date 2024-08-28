package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import java.awt.Font
import java.awt.Graphics2D
import kotlin.math.roundToInt

class Text(
    private val text: String,
    val x: Double,
    val y: Double,
    private val font: String, // 12px Arial for example
    private val theta: Double,
    val stroke: Boolean,
    private val usePageFrame: Boolean,
) : CanvasOpImpl(Type.TEXT) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        val g2d = setup(ctx, scale)
        g2d.font = getFont(scale.scale)
        g2d.drawString(text, x.toFloat(), y.toFloat())
    }

    fun setup(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale): Graphics2D {
        val g2d = ctx.g2d.create() as Graphics2D
        val transform = CanvasTransform(theta, x, y, usePageFrame)
        g2d.transform = getTransform(ctx, scale, transform)
        setColor(ctx, stroke)
        return g2d
    }

    fun getFont(scale: Double): Font {
        val size = (font.substringBefore("px").toDouble() * scale).roundToInt()
        val fontFamily = font.substringAfter("px").trim()
        return Font(fontFamily, Font.PLAIN, size)
    }
}