package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import com.millburnx.utils.vec2d.Vec2d
import java.awt.Graphics2D
import java.awt.image.BufferedImage
import java.io.File
import javax.imageio.ImageIO

class Image(
    private val path: String,
    val x: Double,
    val y: Double,
    private val width: Double,
    private val height: Double,
    private val theta: Double,
    private val pivotX: Double,
    private val pivotY: Double,
    private val usePageFrame: Boolean,
) : CanvasOpImpl(Type.IMAGE) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        val image = getImage()
        val g2d = setup(ctx, scale)
        val topLeft = scale.ppiScale(Vec2d(x to y))
        val size = scale.ppiScale(Vec2d(width to height))
        g2d.drawImage(image, topLeft.x, topLeft.y, size.x, size.y, null)
        g2d.dispose()
    }

    fun getImage(): BufferedImage {
        return ImageIO.read(File(path))
    }

    fun setup(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale): Graphics2D {
        val g2d = ctx.g2d.create() as Graphics2D
        val transform = CanvasTransform(theta, pivotX, pivotY, usePageFrame)
        g2d.transform = getTransform(ctx, scale, transform)
        return g2d
    }
}