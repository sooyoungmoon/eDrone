package org.etri.eDrone;

import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;

public class Util {
	public static void pretty_columns(TableViewer table_viewer, int current_colum_widht) {
		Table table = table_viewer.getTable();
		TableColumn[] columns = table_viewer.getTable().getColumns();

		if (table.getColumnCount() == 0)
			return;
		int width = table.getSize().x / table.getColumnCount() - (4 - table.getColumnCount());
		if (width < 100) {
			width = current_colum_widht;
		} else {
			current_colum_widht = width;
		}
		for (TableColumn column : columns) {
			column.setWidth(width);
		}
	}

}
