package org.etri.eDroneView.Service;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDroneModel.Model_Base;

public class Dialog_ParamOut extends TitleAreaDialog {

	public Label label_1;
	public Label label_2;

	public Text text_1;

	Model_Base model;

	public Dialog_ParamOut(Shell parentShell, Model_Base model) {
		super(parentShell);
		this.model = model;

	}

	@Override
	public void create() {
		super.create();
		setTitle("API 우선순위 설정");
		setMessage("API 우선순위 값을 설정하세요", IMessageProvider.INFORMATION);
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {

		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
	}

	@Override
	protected Control createDialogArea(Composite parent) {
		getShell().setMinimumSize(400, 300);
		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		container.setLayout(layout);
		layout.marginTop = 20;
		layout.marginBottom = 10;
		layout.marginLeft = 20;
		layout.marginRight = 20;
		Label lbtFirstName = new Label(container, SWT.NONE);
		lbtFirstName.setText("API Name");

		GridData gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_1 = new Label(container, SWT.NONE);
		label_1.setLayoutData(gd);

		Label lbtFirstName2 = new Label(container, SWT.NONE);
		lbtFirstName2.setText("Project");

		gd = new GridData();
		gd.grabExcessHorizontalSpace = true;
		gd.horizontalAlignment = GridData.FILL;

		label_2 = new Label(container, SWT.NONE);
		label_2.setLayoutData(gd);

		Label lbtFirstName3 = new Label(container, SWT.NONE);
		lbtFirstName3.setText("Description");

		gd = new GridData(GridData.FILL_BOTH);

		text_1 = new Text(container, SWT.BORDER | SWT.MULTI | SWT.WRAP);
		text_1.setLayoutData(gd);

		label_1.setText(model.name);
		label_2.setText(model.getProjectParent().name);
		text_1.setText(model.description);
		return area;
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void okPressed() {

		model.description = text_1.getText();

		super.okPressed();
	}

}