package com.millburnx.pathplanner.ui.components.editor.row

import javax.swing.AbstractListModel
import javax.swing.JLabel
import kotlin.math.max


class RowModel : AbstractListModel<Row>() {
    val title = Row(JLabel("X:"), JLabel("Y:"), JLabel("Z:"))

    val rows = mutableListOf<Row>(
        Row(JLabel("1"), JLabel("2"), JLabel("LABEL 1")),
        Row(JLabel("LABEL 2"), JLabel("3"), JLabel("4")),
        Row(JLabel("5"), JLabel("REALLY LONG LABEL"), JLabel("6")),
    )

    val maxWidths: MutableList<Int> = mutableListOf()

    val cellSizeListener = CellSizeListener()

    inner class CellSizeListener : java.awt.event.ComponentAdapter() {
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

    fun updateCellSizes() {
        updateMaxWidths()
        updateCellMargins()
    }

    fun updateMaxWidths() {
        maxWidths.clear()
        maxWidths += MutableList(title.children.size) {
            val titleWidth = title.children[it].preferredSize.width
            val columnWidth = rows.map { row -> row.children[it].preferredSize.width }.maxOrNull() ?: 0
            max(titleWidth, columnWidth)
        }
    }

    fun updateCellMargins() {
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

    fun addRow(row: Row, index: Int = rows.size) {
        rows.add(index, row)
        rows.forEachIndexed { rowIndex, row ->
            row.index = rowIndex
        }
        fireIntervalAdded(this, index, index)
    }

    fun removeRow(index: Int) {
        rows.removeAt(index)
        rows.forEachIndexed { rowIndex, row ->
            row.index = rowIndex
        }
        fireIntervalRemoved(this, index, index)
    }
}
