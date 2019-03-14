package org.etri.eDroneWizard;

import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;

public class Table_ParamOut {

	private TableViewer viewer;

	private TableViewerColumn createTableViewerColumn(String title, int bound, final int colNumber) {

		int align = SWT.CENTER;

		final TableViewerColumn viewerColumn = new TableViewerColumn(viewer, align);
		final TableColumn column = viewerColumn.getColumn();
		column.setText(title);
		column.setWidth(bound);
		column.setResizable(true);
		column.setMoveable(true);
		return viewerColumn;
	}

	private void createColumns2(final Composite parent, final TableViewer viewer) {

		String[] titles = { "Type", "Name", "Description" };
		int[] bounds = { 200, 200, 350 };

		TableViewerColumn col = createTableViewerColumn(titles[0], bounds[0], 0);
		col.setLabelProvider(new ColumnLabelProvider() {

			@Override
			public String getText(Object element) {
				return ((Model_Base) element).type;
			}
		});

		col = createTableViewerColumn(titles[1], bounds[1], 1);
		col.setLabelProvider(new ColumnLabelProvider() {

			@Override
			public String getText(Object element) {
				return ((Model_Base) element).name;
			}
		});

		col = createTableViewerColumn(titles[2], bounds[2], 2);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base model = (Model_Base) element;
				return model.description;
			}
		});

	}

	public Table_ParamOut(Composite c) {

		viewer = new TableViewer(c, SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);
		createColumns2(c, viewer);

		final Table table = viewer.getTable();

		table.setLinesVisible(true);
		table.setHeaderVisible(true);

		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 7;
		gd.heightHint = 120;
		gd.grabExcessHorizontalSpace = true;
		viewer.getControl().setLayoutData(gd);
		viewer.setContentProvider(new ArrayContentProvider());

		Global.wizard_page2.tableviewer_paramout = viewer;
	}

}
