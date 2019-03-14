package org.etri.eDroneView.Service;

import java.util.HashMap;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Vector;
import org.etri.eDroneModel.VectorInfo;

public class Dialog_Vector extends Dialog {

	Model_Vector model;
	Text text;

	TableViewer v;
	HashMap<TableColumn, Text> hmap = new HashMap<TableColumn, Text>();
	HashMap<TableColumn, VectorInfo> vi_hmap;

	public Dialog_Vector(Shell parentShell, Model_Vector m, TableViewer v, HashMap<TableColumn, VectorInfo> hmap) {
		super(parentShell);
		this.model = m;
		this.v = v;
		this.vi_hmap = hmap;
	}

	@Override
	public void create() {
		super.create();
	}

	@Override
	protected void configureShell(Shell newShell) {
		// TODO Auto-generated method stub
		super.configureShell(newShell);
		newShell.setText("백터 생성/편집");
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

		for (TableColumn col : v.getTable().getColumns()) {
			GridData gd = new GridData(GridData.FILL_HORIZONTAL);
			Label label = new Label(container, SWT.NULL);
			label.setLayoutData(gd);
			label.setText(col.getText());

			Text text_value = new Text(container, SWT.BORDER | SWT.SINGLE);
			text_value.setLayoutData(gd);

			String s = model.hmap_value.get(col.getText());
			if (s != null) {
				text_value.setText(s);
			}
			hmap.put(col, text_value);
		}

		return area;
	}

	@Override
	protected void okPressed() {

		for (TableColumn col : hmap.keySet()) {

			String value = hmap.get(col).getText();

			VectorInfo vi = vi_hmap.get(col);

			String name = vi.name;
			String min = vi.min;
			String max = vi.max;
			String not = vi.not;

			String converted_type = Global.hashmap_type.get(vi.type).beConverted;

			switch (converted_type) {
			case "int":
				Integer v = 0;
				try {
					v = Integer.parseInt(value);
				} catch (NumberFormatException e) {
					MessageDialog.openError(getShell(), "Warning", name + ": 값이 숫자가 아닙니다");
					return;
				}

				Integer compare = -1;

				if (min.length() > 0) {

					compare = Integer.parseInt(min);

					if (compare > v) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최소값 (" + min + ") 미만입니다");
						return;
					}

				}
				if (max.length() > 0) {
					compare = Integer.parseInt(max);

					if (compare < v) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최대값 (" + max + ") 초과입니다");
						return;
					}
				}
				if (not.length() > 0) {
					compare = Integer.parseInt(not);

					if (compare.equals(v)) {
						MessageDialog.openError(getShell(), "Warning", name + ": 입력 제한값 (" + not + ") 입니다");
						return;
					}
				}

				break;
			case "double":
				Double d = 0.0;
				try {
					d = Double.parseDouble(value);
				} catch (NumberFormatException e) {
					MessageDialog.openError(getShell(), "Warning", "값이 타입에 맞지 않습니다");
					return;
				}

				Double compared = 0.0;

				if (min.length() > 0) {

					compared = Double.parseDouble(min);

					if (compared > d) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최소값 (" + min + ") 미만입니다");
						return;
					}

				}
				if (max.length() > 0) {
					compared = Double.parseDouble(max);

					if (compared < d) {
						MessageDialog.openError(getShell(), "Warning", name + ": 최대값 (" + max + ") 초과입니다");
						return;
					}
				}
				if (not.length() > 0) {
					compared = Double.parseDouble(not);

					if (compared.equals(d)) {
						MessageDialog.openError(getShell(), "Warning", name + ": 입력 제한값 (" + not + ") 입니다");
						return;
					}
				}

				break;
			default:

				if (not.length() > 0) {
					compared = Double.parseDouble(not);

					if (not.equals(value)) {
						MessageDialog.openError(getShell(), "Warning", name + ": 입력 제한값 (" + not + ") 입니다");
						return;
					}
				}
				break;
			}

		}

		for (TableColumn col : hmap.keySet()) {
			model.hmap_value.put(col.getText(), hmap.get(col).getText());
		}

		super.okPressed();
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

}