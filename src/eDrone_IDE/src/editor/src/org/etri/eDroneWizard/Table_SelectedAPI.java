package org.etri.eDroneWizard;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.CheckboxCellEditor;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.EditingSupport;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.RGBA;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableItem;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;
import org.etri.eDroneModel.ParamType;

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

public class Table_SelectedAPI {

	private TableViewer tableviewer_selected_API = null;
	private Composite parent = null;
	private static Model_Base model_current_selected = null;

	public Table_SelectedAPI(Composite parent) {

		this.parent = parent;

		tableviewer_selected_API = new TableViewer(parent, SWT.MULTI | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		tableviewer_selected_API.setContentProvider(new ContentProvider_API_List());

		createColumns(tableviewer_selected_API);

		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.grabExcessHorizontalSpace = true;
		gd.heightHint = 200;
		gd.widthHint = 400;
		gd.verticalSpan = 2;

		final Table table = tableviewer_selected_API.getTable();
		table.setLayoutData(gd);
		table.setHeaderVisible(true);
		table.setLinesVisible(true);
		table.setEnabled(true);

		table.addListener(SWT.PaintItem, new Listener() {

			@Override
			public void handleEvent(Event event) {
				if (event.index == 4) {
					Image tmpImage = null;
					int tmpWidth = 0;
					int tmpHeight = 0;
					int tmpX = 0;
					int tmpY = 0;

					tmpWidth = table.getColumn(event.index).getWidth();
					tmpHeight = ((TableItem) event.item).getBounds().height;
					TableItem item = (TableItem) event.item;
					Model_Base model = (Model_Base) item.getData();
					if (model.isDefaultParam) {
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

		Global.wizard_page2.tableviewer_selected_api = tableviewer_selected_API;
		tableviewer_selected_API.setInput(new Model_Service());
		tableviewer_selected_API.addSelectionChangedListener(new ISelectionChangedListener() {

			@Override
			public void selectionChanged(SelectionChangedEvent event) {
				IStructuredSelection selection = (IStructuredSelection) event.getSelection();

				if (selection == null || selection.isEmpty())
					return;

				Model_Base model = (Model_Base) selection.getFirstElement();
				model_current_selected = model;
				setInputParameters();

			}
		});
	}

	public static void setInputParameters() {

		Model_Base model = model_current_selected;

		Global.wizard_page2.text_API_description.setText(model.description);
		Global.wizard_page2.tableviewer_paramin.getTable().clearAll();
		Global.wizard_page2.tableviewer_paramout.getTable().clearAll();

		List<Model_Base> in_list = new ArrayList<Model_Base>();
		List<Model_Base> out_list = new ArrayList<Model_Base>();
		for (Model_Base m : model.child) {

			if (m.ptype == null) {
				if (m.type.indexOf("[]") != -1) {
					m.ptype = ParamType.Vector;

				} else {

					if (Global.hashmap_type.containsKey(m.type)) {
						m.ptype = ParamType.Text;
					} else {
						m.ptype = ParamType.Class;
					}
				}
			}
			if (m.isIn) {

				in_list.add(m);
			} else {
				out_list.add(m);
			}
		}

		Model_Base[] in_arr = new Model_Base[in_list.size()];
		in_list.toArray(in_arr);
		Global.wizard_page2.tableviewer_paramin.setInput(in_arr);
//		Table_ParamIn.packColumns();

		Model_Base[] out_arr = new Model_Base[out_list.size()];
		out_list.toArray(out_arr);
		Global.wizard_page2.tableviewer_paramout.setInput(out_arr);

	}

	private void createColumns(TableViewer viewer) {

		TableViewerColumn colOrder = new TableViewerColumn(viewer, SWT.BORDER);
		colOrder.getColumn().setWidth(30);
		colOrder.getColumn().setText(" ");
		colOrder.getColumn().setAlignment(SWT.CENTER);
		colOrder.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {

				Model_Base p = (Model_Base) element;
				return " " + p.getOrder();
			}

			@Override
			public Color getForeground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.isDefaultParam) {
					return new Color(parent.getDisplay(), new RGBA(0, 0, 128, 255));
				}

				return null;
			}

		});

		TableViewerColumn colName = new TableViewerColumn(viewer, SWT.BORDER);
		colName.getColumn().setWidth(180);
		colName.getColumn().setText("Name");
		colName.getColumn().setAlignment(SWT.CENTER);
		colName.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {

				Model_Base p = (Model_Base) element;
				return p.toString();
			}

			@Override
			public Color getForeground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.isDefaultParam) {
					return new Color(parent.getDisplay(), new RGBA(0, 0, 128, 255));
				}

				return null;
			}

		});

		TableViewerColumn colDepend = new TableViewerColumn(viewer, SWT.BORDER);
		colDepend.getColumn().setWidth(180);
		colDepend.getColumn().setText("Project");
		colDepend.getColumn().setAlignment(SWT.CENTER);
		colDepend.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {

				Model_Base p = (Model_Base) element;
				return p.getProjectParent().toString();
			}

			@Override
			public Color getForeground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.isDefaultParam) {
					return new Color(parent.getDisplay(), new RGBA(0, 0, 128, 255));
				}

				return null;
			}

		});

		TableViewerColumn colPriority = new TableViewerColumn(viewer, SWT.BORDER);
		colPriority.getColumn().setWidth(70);
		colPriority.getColumn().setText("Priority");
		colPriority.getColumn().setAlignment(SWT.CENTER);
		colPriority.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {

				Model_Base p = (Model_Base) element;

				return Double.toString(p.priority);
			}

			@Override
			public Color getForeground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.isDefaultParam) {
					return new Color(parent.getDisplay(), new RGBA(0, 0, 128, 255));
				}
				return null;
			}

		});

		TableViewerColumn colDefault = new TableViewerColumn(viewer, SWT.BORDER);
		colDefault.getColumn().setWidth(55);
		colDefault.getColumn().setText("Default");
		colDefault.getColumn().setAlignment(SWT.CENTER);
		colDefault.setEditingSupport(new ArgCheckEditingSupport(viewer));
		colDefault.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "   ";
			}

		});

	}

	public static class ArgCheckEditingSupport extends EditingSupport {

		@SuppressWarnings("unused")
		private final TableViewer viewer;

		public ArgCheckEditingSupport(TableViewer viewer) {
			super(viewer);
			this.viewer = viewer;
		}

		@Override
		protected CellEditor getCellEditor(Object element) {
			return new CheckboxCellEditor(null, SWT.CHECK | SWT.READ_ONLY);
		}

		@Override
		protected boolean canEdit(Object element) {
			return true;
		}

		@Override
		protected Object getValue(Object element) {
			Model_Base p = (Model_Base) element;
			return p.isDefaultParam;

		}

		@Override
		protected void setValue(Object element, Object value) {
		}
	}

	private class ContentProvider_API_List implements IStructuredContentProvider {

		public Object[] getElements(Object input) {
			return ((Model_Service) input).api_list.toArray();
		}

	}
}
