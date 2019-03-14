package org.etri.eDroneView.Service;

import java.util.ArrayList;
import java.util.HashMap;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Combo;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Vector;
import org.etri.eDroneModel.VectorInfo;

public class Dialog_Gen_VColumn extends Dialog {

	TableColumn column;
	Text text_name;
	String name;
	ArrayList<Model_Vector> list;
	HashMap<TableColumn, VectorInfo> hmap;
	private boolean isNew;
	Combo combo_type;

	private Text text_min;
	private Text text_max;
	private Text text_not;

	public Dialog_Gen_VColumn(Shell parentShell, TableColumn column, ArrayList<Model_Vector> list,
			HashMap<TableColumn, VectorInfo> hmap, boolean isNew) {
		super(parentShell);
		this.column = column;
		this.name = column.getText();
		this.list = list;
		this.hmap = hmap;
		this.isNew = isNew;
	}

	@Override
	public void create() {
		super.create();
	}

	@Override
	protected void configureShell(Shell newShell) {
		super.configureShell(newShell);
		newShell.setText("멤버 변수 (컬럼) 생성/편집");
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {
		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
	}

	@Override
	protected Control createDialogArea(Composite parent) {
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		container.setLayout(layout);

		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		;
		Label label = new Label(container, SWT.NULL);
		label.setLayoutData(gd);
		label.setText("Type");

		combo_type = new Combo(container, SWT.READ_ONLY);

		String items[] = new String[Global.hashmap_type.keySet().size()];

		int i = 0;
		for (String str : Global.hashmap_type.keySet()) {
			items[i] = str;
			i++;
		}

		combo_type.setItems(items);
		combo_type.setLayoutData(gd);

		int j = 0;
		if (hmap.containsKey(column)) {

			String type = hmap.get(column).type;
			for (String st : items) {
				if (st.equals(type)) {
					combo_type.select(j);
					break;
				}
				j++;
			}
		}

		gd = new GridData(GridData.FILL_HORIZONTAL);
		label = new Label(container, SWT.NULL);
		label.setLayoutData(gd);
		label.setText("Name");

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 1;
		text_name = new Text(container, SWT.BORDER);
		text_name.setLayoutData(gd);
		text_name.setText(column.getText());

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		layout = new GridLayout(2, false);

		Group g_condition = new Group(container, SWT.BORDER);
		g_condition.setLayout(layout);
		g_condition.setLayoutData(gd);
		g_condition.setText("Condition");

		GridData normal_gd = new GridData();
		GridData hfill_gd = new GridData(GridData.FILL_HORIZONTAL);

		label = new Label(g_condition, SWT.NULL);
		label.setLayoutData(normal_gd);
		label.setText("min : ");

		text_min = new Text(g_condition, SWT.SINGLE | SWT.BORDER);
		text_min.setLayoutData(hfill_gd);

		label = new Label(g_condition, SWT.NULL);
		label.setLayoutData(normal_gd);
		label.setText("max : ");

		text_max = new Text(g_condition, SWT.SINGLE | SWT.BORDER);
		text_max.setLayoutData(hfill_gd);

		label = new Label(g_condition, SWT.NULL);
		label.setLayoutData(normal_gd);
		label.setText("not : ");

		text_not = new Text(g_condition, SWT.SINGLE | SWT.BORDER);
		text_not.setLayoutData(hfill_gd);

		checkType();
		fillText();

		return area;
	}

	private void checkType() {

		VectorInfo vi = hmap.get(column);

		if (vi == null)
			return;

		String type = vi.type;

		if (type.length() > 0) {
			String tobeType = Global.hashmap_type.get(type).beConverted;

			if (tobeType.equals("int") || tobeType.equals("double") || tobeType.equals("uint")) {

			} else {
				text_min.setEnabled(false);
				text_max.setEnabled(false);
			}

			for (int i = 0; i < combo_type.getItemCount(); i++) {
				String point = combo_type.getItem(i).toString();
				if (type.equals(point)) {
					combo_type.select(i);
					break;
				}
			}

		}

	}

	private void fillText() {

		VectorInfo vi = hmap.get(column);

		if (vi == null)
			return;

		String min = vi.min;
		String max = vi.max;
		String not = vi.not;

		if (min.length() > 0) {
			text_min.setText(min);
		}
		if (max.length() > 0) {
			text_max.setText(max);
		}
		if (not.length() > 0) {
			text_not.setText(not);
		}

	}

	@Override
	protected void okPressed() {

		String changed = text_name.getText();

		VectorInfo vi = hmap.get(column);

		if (vi == null) {
			vi = new VectorInfo("", changed);
			hmap.put(column, vi);
		}
		vi.type = combo_type.getText();

		if (changed.replace(" ", "").length() > 0) {
			column.setText(changed);

			vi.name = text_name.getText();
			vi.type = combo_type.getText();
			vi.min = text_min.getText().replace(" ", "");
			vi.max = text_max.getText().replace(" ", "");
			vi.not = text_not.getText().replace(" ", "");

			if (name.length() > 0) {
				for (Model_Vector mv : list) {

					String tmp = mv.hmap_value.remove(name);
					mv.hmap_value.put(changed, tmp);

				}
			} else {
				for (Model_Vector mv : list) {
					mv.hmap_value.put(changed, "");
				}
			}

		} else {
			MessageDialog.openError(getShell(), "Warning", "이름 항목은 비어있을 수 없습니다");
		}

		super.okPressed();
	}

	@Override
	protected void cancelPressed() {
		if (isNew) {
			column.dispose();
		}
		super.cancelPressed();
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void buttonPressed(int buttonId) {
		if (IDialogConstants.OK_ID == buttonId) {
			okPressed();
		} else if (IDialogConstants.CANCEL_ID == buttonId) {
			cancelPressed();
		} else if (IDialogConstants.CLOSE_ID == buttonId) {
			super.cancelPressed();
		}
	}

}