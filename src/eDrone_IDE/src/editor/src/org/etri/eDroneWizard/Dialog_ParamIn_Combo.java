package org.etri.eDroneWizard;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.ComboViewer;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDroneModel.Model_Base;

public class Dialog_ParamIn_Combo extends TitleAreaDialog {

	public Label label_type;
	public Label label_name;
	public Label label_3;
	public Label label_4;
	public Label label_5;
	public Label label_6;

	public Text text_2;
	public Text text_3;
	public Text text_4;

	ComboViewer combo_viewer;
	private Model_Base model;
	private Text text_description;
	private Button button_arg;

	public Dialog_ParamIn_Combo(Shell parentShell, Model_Base model) {
		super(parentShell);
		this.model = model;

	}

	@Override
	public void create() {
		super.create();
		setTitle("일반타입 파라미터 설정");
		setMessage("파라미터 입력을 텍스트 혹은 콤보박스 형태로 설정할 수 있습니다", IMessageProvider.INFORMATION);
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
		layout.marginTop = 20;
		layout.marginBottom = 10;
		layout.marginLeft = 20;
		layout.marginRight = 20;
		GridData gd = new GridData();
		Label label = new Label(container, SWT.NULL);
		label.setText("Argument?");
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

		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		gd.heightHint = 120;
		gd.grabExcessHorizontalSpace = true;

		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING);
		gd.horizontalSpan = 2;
		label = new Label(container, SWT.NULL);
		label.setText("Value");
		label.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		combo_viewer = new ComboViewer(container);
		combo_viewer.getCombo().setLayoutData(gd);

		combo_viewer.setContentProvider(new ArrayContentProvider());
		initialize();
		return area;
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

		String back_value = model.back_value;

		if (back_value.length() > 0) {

			String tokens[] = back_value.split(",");
			combo_viewer.setInput(tokens);

			combo_viewer.setSelection(new StructuredSelection(model.value));

		}

	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void okPressed() {

		if (button_arg.getSelection()) {
			model.isArg = true;
		} else {
			model.isArg = false;
		}
		model.description = text_description.getText();
		String str = (String) combo_viewer.getStructuredSelection().getFirstElement();
		model.value = str;
		super.okPressed();
	}

}