package com.millburnx.pathPlanner.components.editor.row

import javax.swing.AbstractListModel
import javax.swing.JLabel
import kotlin.math.max

public class RowModel : AbstractListModel<Row>() {
    public val title: Row = Row(JLabel("X:"), JLabel("Y:"), JLabel("Z:"))

    public val rows: MutableList<Row> = mutableListOf<Row>(
        Row(JLabel("1"), JLabel("2"), JLabel("LABEL 1")),
        Row(JLabel("LABEL 2"), JLabel("3"), JLabel("4")),
        Row(JLabel("5"), JLabel("REALLY LONG LABEL"), JLabel("6")),
    )

    public val maxWidths: MutableList<Int> = mutableListOf()

    public val cellSizeListener: CellSizeListener = CellSizeListener()

    public inner class CellSizeListener : java.awt.event.ComponentAdapter() {
        override fun componentResized(e: java.awt.event.ComponentEvent) {
            updateCellSizes()
        }
    }

    init {
        updateCellSizes()
        title.components.forEach { component ->
            component.addComponentListener(cellSizeListener)
        }
        rows.forEachIndexed { rowIndex, row ->
            row.components.forEach { component ->
                component.addComponentListener(cellSizeListener)
                row.index = rowIndex
            }
        }
    }

    public fun updateCellSizes() {
        updateMaxWidths()
        updateCellMargins()
    }

    public fun updateMaxWidths() {
        maxWidths.clear()
        maxWidths += MutableList(title.children.size) {
            val titleWidth = title.children[it].preferredSize.width
            val columnWidth = rows.map { row -> row.children[it].preferredSize.width }.maxOrNull() ?: 0
            max(titleWidth, columnWidth)
        }
    }

    public fun updateCellMargins() {
        title.updateCellMargins(maxWidths.toList())
        rows.forEach { row ->
            row.updateCellMargins(maxWidths.toList())
        }
    }

    override fun getSize(): Int {
        return rows.size
    }

    override fun getElementAt(index: Int): Row? {
        return rows[index]
    }

    public fun addRow(row: Row, index: Int = rows.size) {
        rows.add(index, row)
        rows.forEachIndexed { rowIndex, row ->
            row.index = rowIndex
        }
        fireIntervalAdded(this, index, index)
    }

    public fun removeRow(index: Int) {
        rows.removeAt(index)
        rows.forEachIndexed { rowIndex, row ->
            row.index = rowIndex
        }
        fireIntervalRemoved(this, index, index)
    }
}