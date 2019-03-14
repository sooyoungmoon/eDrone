package org.etri.eDroneView.Service;

import java.util.ArrayList;
import java.util.HashMap;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ListViewer;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.ViewerDropAdapter;
import org.eclipse.swt.SWT;
import org.eclipse.swt.dnd.DND;
import org.eclipse.swt.dnd.DragSourceEvent;
import org.eclipse.swt.dnd.DragSourceListener;
import org.eclipse.swt.dnd.DropTargetEvent;
import org.eclipse.swt.dnd.TextTransfer;
import org.eclipse.swt.dnd.Transfer;
import org.eclipse.swt.dnd.TransferData;
import org.eclipse.swt.events.ControlEvent;
import org.eclipse.swt.events.ControlListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.RGB;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.TableItem;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDrone.Util;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Vector;
import org.etri.eDroneModel.VectorInfo;

public class Dialog_ParamIn_Vector extends TitleAreaDialog {

	public Label label_type;
	public Label label_name;

	public Text text_value;
	private int minimum_colum_width = 100;
	Button button_text;
	Button button_combo;
	Button button_create;
	Button button_delete;
	ListViewer lv;
	private Model_Base model;
	private Label label_combo;
	private TableViewer table_viewer;
	private Text text_description;
	private Button button_arg;
	private Button button_add_col;
	private Button button_edit_col;
	private Button button_delete_col;
	private Button button_import_csv;
	private Button button_export_csv;

	private boolean isEditable;

	private int selected_column_index = -1;
	private HashMap<TableColumn, VectorInfo> hmap_column = new HashMap<TableColumn, VectorInfo>();

	Composite container;
	ArrayList<Model_Vector> list_Vector = new ArrayList<Model_Vector>();

	public Dialog_ParamIn_Vector(Shell parentShell, Model_Base model, boolean isEditable) {
		super(parentShell);
		this.model = model;
		this.isEditable = isEditable;
	}

	@Override
	public void create() {
		super.create();
		setTitle("백터형 파라미터 설정");
		setMessage("열을 추가하여 멤버변수를 제어할 수 있고, 행을 추가하여 디폴트 값을 지정할 수 있습니다", IMessageProvider.INFORMATION);
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {

		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
	}

	@Override
	protected Control createDialogArea(Composite parent) {

		Composite area = (Composite) super.createDialogArea(parent);
		container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		container.setLayout(layout);
		layout.marginTop = 20;
		layout.marginBottom = 10;
		layout.marginLeft = 20;
		layout.marginRight = 20;
		GridData gd = new GridData();
		Label label = new Label(container, SWT.NULL);
		label.setText("Argument? : ");
		label.setLayoutData(gd);

		button_arg = new Button(container, SWT.CHECK);
		gd = new GridData();
		button_arg.setLayoutData(gd);

		Label lbtFirstName = new Label(container, SWT.NONE);
		lbtFirstName.setText("Type");

		gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_type = new Label(container, SWT.NONE);
		label_type.setLayoutData(gd);

		Label lbtFirstName2 = new Label(container, SWT.NONE);
		lbtFirstName2.setText("Name");

		gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_name = new Label(container, SWT.NONE);
		label_name.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING);
		gd.horizontalSpan = 2;
		label = new Label(container, SWT.NULL);
		label.setText("Description");
		label.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		gd.heightHint = 100;
		text_description = new Text(container, SWT.MULTI | SWT.BORDER | SWT.WRAP);
		text_description.setLayoutData(gd);

		gd = new GridData(GridData.HORIZONTAL_ALIGN_CENTER | GridData.VERTICAL_ALIGN_CENTER);
		gd.horizontalSpan = 2;
		gd.heightHint = 50;
		label_combo = new Label(container, SWT.NULL);
		label_combo.setText("\nVector Type Parameter");
		label_combo.setLayoutData(gd);

//		gd = new GridData();
//		Label dummy = new Label(container, SWT.NULL);
//		dummy.setLayoutData(gd);
		Composite compoiste_buttons = new Composite(container, SWT.NULL);
		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		compoiste_buttons.setLayoutData(gd);

		GridLayout gl = new GridLayout();
		gl.numColumns = 4;
		compoiste_buttons.setLayout(gl);

		GridLayout gll = new GridLayout();
		gll.numColumns = 3;
		gll.marginBottom = 5;
		gd = new GridData(GridData.FILL_BOTH);
		Group g_col = new Group(compoiste_buttons, SWT.BORDER);
		g_col.setLayoutData(gd);
		g_col.setText("Column (멤버변수)");
		g_col.setLayout(gll);

		gd = new GridData(GridData.FILL_BOTH);
		Group g_row = new Group(compoiste_buttons, SWT.BORDER);
		gll = new GridLayout();
		gll.numColumns = 2;
		g_row.setLayoutData(gd);
		g_row.setLayout(gll);
		g_row.setText("Row (순서쌍)");

		gll = new GridLayout();
		gll.numColumns = 3;
		gd = new GridData(GridData.FILL_BOTH);
		Group g_csv = new Group(compoiste_buttons, SWT.BORDER);
		g_csv.setLayoutData(gd);
		g_csv.setLayout(gll);
		g_csv.setText("CSV");

//		gd = new GridData(GridData.FILL_VERTICAL);
//		Button btn_condition = new Button(compoiste_buttons, SWT.PUSH);
//		btn_condition.setLayoutData(gd);
//		btn_condition.setText("Condition");
//		btn_condition.addSelectionListener(new SelectionListener() {
//
//			@Override
//			public void widgetSelected(SelectionEvent e) {
//
//				Dialog_Condition dc = new Dialog_Condition(getShell(), condition_holder);
//				dc.open();
//
//			}
//
//			@Override
//			public void widgetDefaultSelected(SelectionEvent e) {
//				// TODO Auto-generated method stub
//
//			}
//		});

		button_add_col = new Button(g_col, SWT.PUSH);
		button_add_col.setImage(Global.image_Add);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_add_col.setLayoutData(gd);
		button_add_col.addSelectionListener(new SelectionListener() {
			@Override
			public void widgetSelected(SelectionEvent e) {

				TableViewerColumn tvc = createTableViewerColumn(table_viewer, "", 200);
				Dialog_Gen_VColumn dvc = new Dialog_Gen_VColumn(getShell(), tvc.getColumn(), list_Vector, hmap_column,
						true);
				if (dvc.open() == 0) {
					table_viewer.refresh();
					Util.pretty_columns(table_viewer, minimum_colum_width);
				}
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		button_edit_col = new Button(g_col, SWT.PUSH);
		button_edit_col.setImage(Global.image_Service);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_edit_col.setLayoutData(gd);
		button_edit_col.addSelectionListener(new SelectionListener() {
			@Override
			public void widgetSelected(SelectionEvent e) {
				if (selected_column_index == -1) {
					MessageDialog.openError(container.getShell(), "Warning", "선택된 컬럼이 없습니다");
					return;
				}
				Dialog_Gen_VColumn dvc = new Dialog_Gen_VColumn(getShell(),
						table_viewer.getTable().getColumn(selected_column_index), list_Vector, hmap_column, false);
				dvc.open();

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		button_delete_col = new Button(g_col, SWT.PUSH);
		button_delete_col.setImage(Global.image_delete);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_delete_col.setLayoutData(gd);
		button_delete_col.addSelectionListener(new SelectionListener() {
			@Override
			public void widgetSelected(SelectionEvent e) {

				if (selected_column_index == -1) {
					MessageDialog.openError(container.getShell(), "Warning", "선택된 컬럼이 없습니다");
					return;
				}
				if (selected_column_index == 0 && table_viewer.getTable().getColumns().length == 1) {
					table_viewer.getTable().removeAll();
				}

				table_viewer.getTable().getColumn(selected_column_index).dispose();
				Util.pretty_columns(table_viewer, minimum_colum_width);
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});
		button_create = new Button(g_row, SWT.PUSH);
		button_create.setImage(Global.image_Add);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_create.setLayoutData(gd);
		button_create.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				if (table_viewer.getTable().getColumns().length < 1) {
					MessageDialog.openError(container.getShell(), "Warning", "컬럼을 먼저 추가하세요");
					return;
				}

				ArrayList<Model_Vector> list = (ArrayList<Model_Vector>) table_viewer.getInput();
				Model_Vector mv = new Model_Vector();
				Dialog_Vector dv = new Dialog_Vector(getShell(), mv, table_viewer, hmap_column);

				if (dv.open() == 0) {
					list.add(mv);
					table_viewer.refresh();
					Util.pretty_columns(table_viewer, minimum_colum_width);
				}

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		button_delete = new Button(g_row, SWT.PUSH);
		button_delete.setImage(Global.image_delete);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_delete.setLayoutData(gd);
		button_delete.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				IStructuredSelection is = table_viewer.getStructuredSelection();
				if (is.isEmpty()) {
					MessageDialog.openError(container.getShell(), "Warning", "선택된 행이 없습니다");
					return;
				}
				Model_Vector mc = (Model_Vector) is.getFirstElement();

				ArrayList<Model_Vector> list = (ArrayList<Model_Vector>) table_viewer.getInput();
				int i = 0;
				for (; i < list.size(); i++) {
					if (list.get(i).equals(mc))
						break;
				}
				list.remove(mc);

				table_viewer.refresh();
				if (i - 1 >= 0) {
					table_viewer.setSelection(new StructuredSelection(list.get(i - 1)), true);
				}
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		button_import_csv = new Button(g_csv, SWT.PUSH);
		button_import_csv.setImage(Global.image_Import);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_import_csv.setLayoutData(gd);
		button_import_csv.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				// TODO Auto-generated method stub

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
				// TODO Auto-generated method stub

			}

		});

		button_export_csv = new Button(g_csv, SWT.PUSH);
		button_export_csv.setImage(Global.image_Export);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_export_csv.setLayoutData(gd);
		button_export_csv.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				// TODO Auto-generated method stub

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
				// TODO Auto-generated method stub

			}

		});
		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		gd.heightHint = 300;
		table_viewer = new TableViewer(container,
				SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);
		table_viewer.getTable().setLinesVisible(true);
		table_viewer.setContentProvider(new ContentProvider_vector());
		table_viewer.getTable().setHeaderVisible(true);
		table_viewer.addDoubleClickListener(new IDoubleClickListener() {

			@Override
			public void doubleClick(DoubleClickEvent event) {
				IStructuredSelection is = (IStructuredSelection) event.getSelection();
				Model_Vector mc = (Model_Vector) is.getFirstElement();

				Dialog_Vector dc = new Dialog_Vector(getShell(), mc, table_viewer, hmap_column);
				dc.open();

				table_viewer.refresh();
			}

		});

		table_viewer.getTable().setLayoutData(gd);
		table_viewer.getTable().addControlListener(new ControlListener() {

			@Override
			public void controlResized(ControlEvent e) {

				Util.pretty_columns(table_viewer, minimum_colum_width);
			}

			@Override
			public void controlMoved(ControlEvent e) {
			}
		});

		int operations = DND.DROP_MOVE;
		Transfer[] transferTypes = new Transfer[] { TextTransfer.getInstance() };
		table_viewer.addDragSupport(operations, transferTypes, new MyDragListener(table_viewer));
		table_viewer.addDropSupport(operations, transferTypes, new MyDropListener(table_viewer));
		initialize();
		return area;
	}

	public class MyDragListener implements DragSourceListener {

		private final TableViewer viewer;

		public MyDragListener(TableViewer viewer) {
			this.viewer = viewer;
		}

		@Override
		public void dragFinished(DragSourceEvent event) {
			System.out.println("Finshed Drag");
		}

		@Override
		public void dragSetData(DragSourceEvent event) {
			// Here you do the convertion to the type which is expected.

			Integer i = viewer.getTable().getSelectionIndex();
			if (TextTransfer.getInstance().isSupportedType(event.dataType)) {
				event.data = i.toString();
			}

		}

		@Override
		public void dragStart(DragSourceEvent event) {
			System.out.println("Start Drag");
		}

	}

	public class MyDropListener extends ViewerDropAdapter {

		private final TableViewer viewer;
		int location;
		int target_index;
		ArrayList<Model_Vector> list;

		public MyDropListener(TableViewer viewer) {
			super(viewer);
			this.viewer = viewer;
		}

		@Override
		public void drop(DropTargetEvent event) {
			location = this.determineLocation(event);

			Model_Vector target = (Model_Vector) determineTarget(event);

			list = (ArrayList<Model_Vector>) viewer.getInput();

			int i = 0;
			for (; i < list.size(); i++) {

				if (target.equals(list.get(i))) {
					target_index = i;
					break;
				}

			}

			super.drop(event);
		}

		// This method performs the actual drop
		// We simply add the String we receive to the model and trigger a refresh of the
		// viewer by calling its setInput method.
		@Override
		public boolean performDrop(Object data) {

			String payload = data.toString();
			Integer source_index = Integer.parseInt(payload);
			Model_Vector mv;
			switch (location) {
			case 1:
				// "Dropped before the target ";

				if (target_index < source_index) {
					mv = list.get(source_index);
					list.remove(mv);
					list.add(target_index, mv);
					viewer.refresh();
					viewer.setSelection(new StructuredSelection(mv));
				} else if (target_index > source_index) {
					mv = list.get(source_index);
					list.add(target_index, mv);
					list.remove(mv);
					viewer.refresh();
					viewer.setSelection(new StructuredSelection(mv));
				}

				break;
			case 2:
				// "Dropped after the target ";

				if (target_index < source_index) {
					mv = list.get(source_index);
					list.remove(mv);
					list.add(target_index + 1, mv);
					viewer.refresh();
					viewer.setSelection(new StructuredSelection(mv));
				} else if (target_index > source_index) {
					mv = list.get(source_index);
					list.add(target_index + 1, mv);
					list.remove(mv);
					viewer.refresh();
					viewer.setSelection(new StructuredSelection(mv));
				}

				break;
			case 3:
				break;
			case 4:
				break;
			}
//	    	File elif = new File();

			return false;
		}

		@Override
		public boolean validateDrop(Object target, int operation, TransferData transferType) {
			return true;

		}

	}

	private class ContentProvider_vector implements IStructuredContentProvider {

		public Object[] getElements(Object input) {
			ArrayList<Model_Vector> list = (ArrayList<Model_Vector>) input;
			return list.toArray();
		}

	}

	private void initialize() {

		if (model.isArg) {
			button_arg.setSelection(true);
		} else {
			button_arg.setSelection(false);
		}

		label_type.setText(":  " + model.type);
		label_name.setText(":  " + model.name);
		text_description.setText(model.description);

		String payload = model.value;
		String tokens[] = payload.split(";");

		ArrayList<String> list = new ArrayList<String>();

		if (tokens[0].length() > 0) {

			Integer num_columns = Integer.parseInt(tokens[0]);
			int i = 1;
			for (; i < num_columns + 1; i++) {
				String token = tokens[i];
				String mini_tokens[] = token.split(",");

				String type = mini_tokens[0];
				String name = mini_tokens[1];

				String min = "";
				String max = "";
				String not = "";

				if (mini_tokens.length > 2) {
					min = mini_tokens[2];
					max = mini_tokens[3];
					not = mini_tokens[4];
				}
				list.add(name);

				VectorInfo vi = new VectorInfo(type, name);

				vi.min = min.equals("None") || min.length() < 1 ? "" : min;
				vi.max = max.equals("None") || max.length() < 1 ? "" : max;
				vi.not = not.equals("None") || not.length() < 1 ? "" : not;

				TableViewerColumn tbc = createTableViewerColumn(table_viewer, name, 0);
				hmap_column.put(tbc.getColumn(), vi);
			}

			String values = tokens[i];
			if (values.equals("None") == false && values.length() > 0) {
				String token_values[] = values.split(",");

				Model_Vector mv = null;
				for (int j = 0; j < token_values.length; j++) {

					if (j % num_columns == 0 && j != 0) {
						list_Vector.add(mv);
					}
					if (j % num_columns == 0) {
						mv = new Model_Vector();
					}
					mv.hmap_value.put(list.get(j % num_columns), token_values[j]);
				}
				if (mv != null) {
					list_Vector.add(mv);
				}

			}
		}

		table_viewer.setInput(list_Vector);

	}

	private TableViewerColumn createTableViewerColumn(TableViewer viewer, String title, int bound) {

		int align = SWT.CENTER;
		final TableViewerColumn viewerColumn = new TableViewerColumn(viewer, align);

		final TableColumn column = (TableColumn) viewerColumn.getColumn();

		column.setText(title);
		column.setWidth(bound);
		column.setResizable(true);
		column.setMoveable(false);

		viewerColumn.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {

				if (element instanceof Model_Vector) {
					String current_title = column.getText();
					Model_Vector mc = (Model_Vector) element;

					if (mc.hmap_value.containsKey(current_title)) {
						return mc.hmap_value.get(current_title);

					}
				}
				return null;
			}
		});

		column.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				int index = 0;

				table_viewer.getTable().deselectAll();

				int i = 0;
				for (TableColumn tc : table_viewer.getTable().getColumns()) {
					if (tc.equals(column)) {
						index = i;
						break;
					}
					i++;
				}

				// 컬럼 선택 기능이 없는 것으로 판단해서 직접 구현함.
				if (selected_column_index == index) {
					for (TableItem ti : table_viewer.getTable().getItems()) {

						ti.setForeground(index, new Color(getShell().getDisplay(), new RGB(0, 0, 0)));
						ti.setBackground(index, new Color(getShell().getDisplay(), new RGB(247, 247, 247)));
					}
					selected_column_index = -1;
				}

				else {
					// 선택되지 않았었다면 선택

					for (TableItem ti : table_viewer.getTable().getItems()) {
						ti.setBackground(index, new Color(getShell().getDisplay(), new RGB(242, 124, 64)));
						ti.setForeground(index, new Color(getShell().getDisplay(), new RGB(255, 255, 255)));
					}

					if (selected_column_index != -1) {
						for (TableItem ti : table_viewer.getTable().getItems()) {
							ti.setBackground(selected_column_index,
									new Color(getShell().getDisplay(), new RGB(247, 247, 247)));
							ti.setForeground(selected_column_index,
									new Color(getShell().getDisplay(), new RGB(0, 0, 0)));
						}
					}
					selected_column_index = index;

				}
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}

		});

		return viewerColumn;
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	// save content of the Text fields because they get disposed
	// as soon as the Dialog closes

	@Override
	protected void okPressed() {

		TableColumn[] array = table_viewer.getTable().getColumns();

		if (array.length < 1) {
			MessageDialog.openError(container.getShell(), "Warning", "최소 하나의 컬럼이 있어야 합니다");
			return;
		}

		if (button_arg.getSelection()) {
			model.isArg = true;
		} else {
			model.isArg = false;
		}
		model.description = text_description.getText();

		StringBuilder sb = new StringBuilder();

//		model.min = condition_holder.min;
//		model.max = condition_holder.max;
//		model.not = condition_holder.not;

		sb.append(Integer.toString(array.length));
		sb.append(";");

		for (TableColumn tc : array) {

			VectorInfo vi = hmap_column.get(tc);
			String type = vi.type;
			String name = vi.name;

			String min = vi.min.length() < 1 ? "None" : vi.min;
			String max = vi.max.length() < 1 ? "None" : vi.max;
			String not = vi.not.length() < 1 ? "None" : vi.not;

			sb.append(type + "," + name + "," + min + "," + max + "," + not);
			sb.append(";");
		}

		if (list_Vector.size() < 1 || table_viewer.getTable().getItemCount() < 1) {
			sb.append("None");

		} else {

			boolean isFirst = true;

			for (Model_Vector mv : list_Vector) {

				for (TableColumn tc : array) {
					if (isFirst) {
						sb.append(mv.hmap_value.get(tc.getText()));
						isFirst = false;
						continue;
					}
					sb.append(",");
					sb.append(mv.hmap_value.get(tc.getText()));

				}

			}

		}

		System.out.println(sb.toString());
		model.value = sb.toString();

		super.okPressed();
	}

}