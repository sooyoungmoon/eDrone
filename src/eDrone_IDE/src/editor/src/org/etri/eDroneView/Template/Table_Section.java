package org.etri.eDroneView.Template;

import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.TableItem;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Section;

/*
 * 
 * 두번째 페이지 오른쪽 상단 리스트 뷰어를 구성한다.
 *
 * SharedVariables에 static으로 등록되며
 * SharedVariables.getLVSelected();로 어느 클래스에서도 가져올 수 있다.
 * 
 * eDroneWizard.java의 getNextPage()에서  
 * eDroneWizardPage.java의 packDefaultValueAndListSelectedServices()를 호출하고
 * 
 * 그 함수 내에서 
 * SharedVariables.getLVSelected().setInput(new Model_APIList(LSelected));
 * 라는 함수를 통해서 데이터를 공급받는다. 
 * 
 */

public class Table_Section {

	public TableViewer viewer = null;

	public Table_Section(Composite parent) {

		viewer = new TableViewer(parent, SWT.MULTI | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		viewer.setContentProvider(new ArrayContentProvider());

		createColumns(viewer);

		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.heightHint = 150;
		gd.horizontalSpan = 2;

		final Table table = viewer.getTable();
		table.setLayoutData(gd);
		table.setHeaderVisible(false);
		table.setLinesVisible(true);
		table.setEnabled(true);

		table.addListener(SWT.PaintItem, new Listener() {

			@Override
			public void handleEvent(Event event) {
				if (event.index == 0) {
					Image tmpImage = null;
					int tmpWidth = 0;
					int tmpHeight = 0;
					int tmpX = 0;
					int tmpY = 0;

					tmpWidth = table.getColumn(event.index).getWidth();
					tmpHeight = ((TableItem) event.item).getBounds().height;
					TableItem item = (TableItem) event.item;
					Model_Section model = (Model_Section) item.getData();
					if (model.content_stringbuilder.length() > 0) {
						tmpImage = Global.image_Checked;
					} else {
						tmpImage = Global.image_UnChecked;
					}

					tmpX = tmpImage.getBounds().width;
					tmpX = (tmpWidth / 2 - tmpX / 2);
					tmpY = tmpImage.getBounds().height;
					tmpY = (tmpHeight / 2 - tmpY / 2);
					if (tmpX <= 0)
						tmpX = event.x;
					else
						tmpX += event.x;
					if (tmpY <= 0)
						tmpY = event.y;
					else
						tmpY += event.y;
					event.gc.drawImage(tmpImage, tmpX, tmpY);

				}
			}
		});

		// Global.wizard_page2.tableviewer_selected_api = tableviewer_selected_API;

		viewer.getTable().setEnabled(false);
		viewer.addSelectionChangedListener(new ISelectionChangedListener() {

			@Override
			public void selectionChanged(SelectionChangedEvent event) {

				IStructuredSelection selection = (IStructuredSelection) event.getSelection();

				Model_Section model = (Model_Section) selection.getFirstElement();

				if (model == null)
					return;

				Global.view_template.annotation.setText(model.annotation);
				Global.view_template.text_content.setText(model.content_stringbuilder.toString());
			}
		});
	}

	public void pack() {

		TableColumn[] cols = viewer.getTable().getColumns();
		for (int i = 0; i < cols.length; i++) {
			cols[i].pack();
		}
	}

	private void createColumns(TableViewer viewer) {

		TableViewerColumn colHasContents = new TableViewerColumn(viewer, SWT.BORDER);
		colHasContents.getColumn().setWidth(40);
		colHasContents.getColumn().setAlignment(SWT.CENTER);
		colHasContents.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "   ";
			}

		});

		TableViewerColumn colName = new TableViewerColumn(viewer, SWT.BORDER);
		colName.getColumn().setWidth(180);
		colName.getColumn().setText("Section List");
		colName.getColumn().setAlignment(SWT.LEFT);
		colName.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Section p = (Model_Section) element;
				return p.name;
			}
		});

	}
}
