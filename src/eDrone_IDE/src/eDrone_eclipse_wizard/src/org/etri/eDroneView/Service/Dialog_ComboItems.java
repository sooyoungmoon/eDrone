package org.etri.eDroneView.Service;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDroneModel.Model_ComboItem;

public class Dialog_ComboItems extends Dialog {

	Model_ComboItem model;
	Text text;

	public Dialog_ComboItems(Shell parentShell, Model_ComboItem m) {
		super(parentShell);
		this.model = m;
	}

	@Override
	public void create() {
		super.create();
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

		GridData gd = new GridData();
		Label label = new Label(container, SWT.NULL);
		label.setText("Name : ");
		label.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		text = new Text(container, SWT.BORDER);
		text.setLayoutData(gd);
		text.setText(model.name);

		gd = new GridData();
		label = new Label(container, SWT.NULL);
		label.setText("Default?");
		label.setLayoutData(gd);

		Button button_default = new Button(container, SWT.CHECK);
		gd = new GridData();
		button_default.setLayoutData(gd);
		button_default.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				Button b = (Button) e.getSource();
				if (b.getSelection()) {
					model.isDefault = true;
				} else {
					model.isDefault = false;
				}
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		if (model.isDefault) {
			button_default.setSelection(true);
		}

		return area;
	}

	@Override
	protected void okPressed() {
		model.name = text.getText();
		super.okPressed();
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

}