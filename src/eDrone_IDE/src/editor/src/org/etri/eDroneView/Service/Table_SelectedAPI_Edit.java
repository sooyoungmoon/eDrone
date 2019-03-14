package org.etri.eDroneView.Service;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyAdapter;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.RGBA;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Table;
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

public class Table_SelectedAPI_Edit {

	Composite parent;
	public TableViewer viewer;
	private static Model_Base model_current_selected = null;
	private Model_Base Model_Copied = null;

	public Table_SelectedAPI_Edit(Composite parent) {

		this.parent = parent;

		viewer = new TableViewer(parent, SWT.MULTI | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		viewer.setContentProvider(new ContentProvider_API_List());

		createColumns(viewer);

		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.grabExcessHorizontalSpace = true;
		gd.heightHint = 150;
		gd.widthHint = 400;
		gd.verticalSpan = 2;

		final Table table = viewer.getTable();
		table.setLayoutData(gd);
		table.setLayoutData(gd);
		table.setHeaderVisible(true);
		table.setLinesVisible(true);

		table.setEnabled(true);

		Global.dialog_opened.tableviewer_selected_api = viewer;
		viewer.setInput(new Model_Service());

		viewer.addSelectionChangedListener(new ISelectionChangedListener() {

			@Override
			public void selectionChanged(SelectionChangedEvent event) {
				IStructuredSelection selection = (IStructuredSelection) event.getSelection();

				if (selection == null || selection.isEmpty())
					return;

				Model_Base model = (Model_Base) selection.getFirstElement();
				model_current_selected = model;
				setInputParameters();

				Global.dialog_opened.title_description.setText("API 설명 입력 : " + model.name);
				Global.dialog_opened.title_description.pack();
			}

		});
		viewer.addDoubleClickListener(new IDoubleClickListener() {
			@Override
			public void doubleClick(DoubleClickEvent event) {
				IStructuredSelection is = (IStructuredSelection) event.getSelection();
				Model_Base model = (Model_Base) is.getFirstElement();

				if (model == null)
					return;
				System.out.println(model.name);
				Double before = model.priority;
				Dialog_Priority dp = new Dialog_Priority(parent.getShell(), model);

				if (dp.open() == 0) {

					if (before.compareTo(model.priority) != 0) {

						Model_Service ms = (Model_Service) viewer.getInput();
						ArrayList<Model_Base> list = ms.api_list;
						ArrayList<Model_Base> new_list = new ArrayList<Model_Base>();

						list.remove(model);

						boolean inserted = false;
						int i = 0;
						if (list.size() > 0) {

							for (Model_Base mb : list) {
								if (model.priority.compareTo(mb.priority) > 0 && inserted == false) {
									new_list.add(model);
									model.order = i + 1;
									inserted = true;
									i++;
								}
								new_list.add(mb);
								mb.order = i + 1;
								i++;
							}
						}
						if (inserted == false) {
							model.order = i + 1;
							new_list.add(model);
						}
						ms.api_list = new_list;

						viewer.setInput(ms);

					}
				}
			}

		});

		viewer.getTable().addKeyListener(new KeyAdapter() {

			@Override
			public void keyPressed(KeyEvent e) {
				String string = "";
				if ((e.stateMask & SWT.CTRL) != 0) {

					if (e.keyCode == 99) {

						IStructuredSelection is = viewer.getStructuredSelection();
						Model_Copied = (Model_Base) is.getFirstElement();

					} else if (e.keyCode == 118) {
						if (Model_Copied != null) {

							int index = viewer.getTable().getSelectionIndex();
							if (index != -1) {

								Model_Service ms = (Model_Service) viewer.getInput();
								ArrayList<Model_Base> list = ms.api_list;

								Model_Base newModel = new Model_Base(Model_Copied);

								if (index < list.size() - 1) {
									list.add(index + 1, newModel);

								} else {
									list.add(newModel);
								}
								int i = 0;
								for (Model_Base model : list) {
									model.order = i + 1;
									i++;
								}
								viewer.setInput(ms);
							}

						}

					}
				} else if (e.keyCode == 127) {
					IStructuredSelection is = viewer.getStructuredSelection();
					if (is.isEmpty() == false) {

						Model_Base mb = (Model_Base) is.getFirstElement();
						Model_Service ms = (Model_Service) viewer.getInput();
						ArrayList<Model_Base> list = ms.api_list;
						list.remove(mb);
						int i = 0;
						for (Model_Base model : list) {
							model.order = i + 1;
							i++;
						}
						viewer.setInput(ms);

					}
				}

				else {
					string += "CTRL - keyCode = " + e.keyCode;
					System.out.println(string);

				}
				viewer.refresh();
			}

		});

	}

	public static void setInputParameters() {

		Model_Base model = model_current_selected;

		Global.dialog_opened.text_API_description.setText(model.description);
		Global.dialog_opened.tableviewer_paramin.getTable().clearAll();
		Global.dialog_opened.tableviewer_paramout.getTable().clearAll();

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
		Global.dialog_opened.tableviewer_paramin.setInput(in_arr);

		Model_Base[] out_arr = new Model_Base[out_list.size()];
		out_list.toArray(out_arr);
		Global.dialog_opened.tableviewer_paramout.setInput(out_arr);

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
		colName.getColumn().setWidth(170);
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
		colDepend.getColumn().setWidth(150);
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

	}

	private class ContentProvider_API_List implements IStructuredContentProvider {

		public Object[] getElements(Object input) {
			return ((Model_Service) input).api_list.toArray();
		}

	}

}
