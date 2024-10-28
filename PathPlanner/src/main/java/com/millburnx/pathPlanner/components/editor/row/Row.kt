package com.millburnx.pathPlanner.components.editor.row

import com.millburnx.util.geometry.Vec2dI
import java.awt.Color
import java.awt.GridBagConstraints
import java.awt.GridBagLayout
import java.awt.datatransfer.DataFlavor
import java.io.Serializable
import javax.swing.JComponent
import javax.swing.JPanel

public class Row(public val children: List<JComponent>) : JPanel(), Serializable {
    public var index: Int = -1
        set(value) {
            background = if (value % 2 == 0) Color.BLACK else Color.DARK_GRAY
        }

    public companion object {
        public val FLAVOR: DataFlavor = DataFlavor(JPanel::class.java, "JPanel")
    }

    public constructor(vararg children: JComponent) : this(children.toList())

    public val padding: Vec2dI = Vec2dI(12, 8)

    init {
        layout = GridBagLayout()
        children.forEachIndexed { index, component ->
            add(component, getConstraints(index))
        }
    }

    public fun updateCellMargins(maxWidths: List<Int>) {
        children.forEachIndexed { index, component ->
            val diff = maxWidths[index] - component.preferredSize.width
            val gridBag = layout as GridBagLayout
            val constraints = getConstraints(index, diff.coerceAtLeast(0))
            gridBag.setConstraints(component, constraints)
        }
    }

    public fun getConstraints(index: Int, offset: Int = 0): GridBagConstraints {
        val constraints = GridBagConstraints()
        constraints.gridy = 0
        constraints.gridx = index
        constraints.anchor = GridBagConstraints.BASELINE_LEADING
        constraints.weightx = 1.0
        constraints.insets.set(padding.y, padding.x, padding.y, padding.x + offset)
        return constraints
    }
}