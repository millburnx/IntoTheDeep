package com.millburnx.util.javafx

import com.millburnx.util.geometry.Vec2d
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Graphics2D
import java.awt.Insets
import java.awt.Point
import java.awt.RenderingHints
import java.awt.Toolkit
import java.awt.image.BufferedImage
import javax.swing.JSpinner
import javax.swing.SpinnerNumberModel
import kotlin.math.ceil
import kotlin.math.floor

public object Utils {
    public fun bufferedImage(width: Int, height: Int): Pair<BufferedImage, Graphics2D> {
        val bufferedImage = BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB)
        val g2d = bufferedImage.createGraphics()

        val desktopHints = Toolkit.getDefaultToolkit().getDesktopProperty("awt.font.desktophints")

        g2d.setRenderingHints(RenderingHints(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON))
        g2d.addRenderingHints(RenderingHints(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE))
        g2d.addRenderingHints(
            RenderingHints(
                RenderingHints.KEY_TEXT_ANTIALIASING,
                RenderingHints.VALUE_TEXT_ANTIALIAS_ON
            )
        )
        desktopHints?.let { g2d.addRenderingHints(desktopHints as RenderingHints) }

        return Pair(bufferedImage, g2d)
    }

    public fun drawRoundedPanel(
        g2d: Graphics2D, scale: Double, size: Vec2d, bgColor: Color, radius: Double,
        borderColor: Color = Color.WHITE, borderWidth: Double = 0.0
    ) {
        val radius = (radius * scale).toInt()

        g2d.color = bgColor
        g2d.fillRoundRect(0, 0, size.x.toInt(), size.y.toInt(), radius, radius)

        if (borderWidth <= 0) return
        val borderWidth = (borderWidth * scale).toFloat()

        g2d.stroke = BasicStroke(borderWidth)
        g2d.color = borderColor
        g2d.drawRoundRect(
            ceil(borderWidth / 2).toInt(),
            ceil(borderWidth / 2).toInt(),
            floor(size.x - borderWidth).toInt(),
            floor(size.y - borderWidth).toInt(),
            radius,
            radius
        )
    }

    public fun jNumber(
        range: ClosedFloatingPointRange<Double>,
        step: Double = 1.0,
        startingValue: Double = 0.0.coerceIn(range),
    ): JSpinner {
        return JSpinner(SpinnerNumberModel(startingValue, range.start, range.endInclusive, step))
    }
}

public fun Vec2d.toAwtPoint(): Point = Point(x.toInt(), y.toInt())
public fun Vec2d.toInsets(): Insets = Insets(y.toInt(), x.toInt(), y.toInt(), x.toInt())