package org.etri.eDroneWizard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.wizard.IWizardPage;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;

/**
 * @Class WizardPage2
 * @brief API 편집 및 파라미터 값 설정 페이지를 구현한 클래스
 * 
 */

public class WizardPage2 extends WizardPage implements IWizardPage {

	public static String projectName;

	// 두번째 페이지 가용 API 트리의 static 변수
	public TreeViewer treeviewer_availabe_api;
	// 두번 페이지 선택된 API 테이블의 static 변수
	public TableViewer tableviewer_selected_api;
	// 두번째 페이지 선택된 API 중 클릭된 API의 입력 파라미터를 보여주는 테이블의 static 변수
	public TableViewer tableviewer_paramin;
	// 두번째 페이지 선택된 API 중 클릭된 API의 출력 파라미터를 보여주는 테이블의 static 변수
	public TableViewer tableviewer_paramout;
	// 두번째 페이지 상단 두번째 텝에 선택된 API들의 의존 프로젝트를 보여주는 테이블의 static 변수
	public TableViewer tableviwer_dependencies;

	public Text text_API_description;
	public Label infoLabel;

	private GridLayout gl;
	private GridData gd;

	public static HashMap<String, HashMap<String, String>> map = new HashMap<String, HashMap<String, String>>();

	Composite container;
	@SuppressWarnings("unused")
	private ISelection selection;

	public WizardPage2(ISelection selection) {
		super("wizardPage");
		setTitle("eDrone Project");
		setDescription("Finish 버튼을 눌러 프로젝트를 생성하세요");
		this.selection = selection;
	}

	/*
	 * createControl UI가 배치되고 tab 클래스들이 불려진다.
	 */

	@Override
	public void createControl(Composite parent) {

		container = new Composite(parent, SWT.NULL);
		gl = new GridLayout(3, false);
		gl.numColumns = 3;
		container.setLayout(gl);

		TabFolder tf = new TabFolder(container, SWT.NULL | SWT.V_SCROLL);
		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 3;
		gd.heightHint = 500;
		gd.grabExcessHorizontalSpace = true;
		gd.grabExcessVerticalSpace = true;
		tf.setLayoutData(gd);

		TabItem ti3 = new TabItem(tf, SWT.BORDER);
		ti3.setText("API 호출 관리");
		ti3.setControl(new tabService(tf, SWT.SHADOW_ETCHED_IN));

		TabItem ti2 = new TabItem(tf, SWT.BORDER);
		ti2.setText("의존 정보 조회");
		ti2.setControl(new tabDependencies(tf, SWT.SHADOW_ETCHED_IN));

		setControl(container);
		setPageComplete(false);
	}

	@Override
	public IWizardPage getPreviousPage() {
		Global.wizard_page1.treeviewer_service.setSelection(Global.wizard_page1.treeviewer_service.getSelection());
		return super.getPreviousPage();
	}

	/*
	 * API 탭 클래스
	 */

	private class tabService extends Composite {

		tabService(Composite c, int style) {
			super(c, style);
			gl = new GridLayout(3, false);
			gl.numColumns = 4;
			this.setLayout(gl);

			gl = new GridLayout();
			GridData gd = new GridData(GridData.FILL_BOTH);
			gd.verticalSpan = 2;
			Group g_availabe = new Group(this, SWT.BORDER);
			g_availabe.setText("가용 API 리스트");
			g_availabe.setLayout(gl);
			g_availabe.setLayoutData(gd);

			new Tree_AvailableAPIs(g_availabe);

			createArrowButton(this, SWT.RIGHT, sel_listener_button_move_right, 1);
			gl = new GridLayout();
			gl.numColumns = 2;

			gd = new GridData(GridData.FILL_BOTH);
			gd.horizontalSpan = 2;
			gd.verticalSpan = 2;
			Group g_selected = new Group(this, SWT.BORDER);
			g_selected.setText("선택된 API 리스트 및 정보");
			g_selected.setLayout(gl);
			g_selected.setLayoutData(gd);

			new Table_SelectedAPI(g_selected);

			createArrowButton(g_selected, SWT.UP, sel_listener_move_up, 2);

			createArrowButton(this, SWT.LEFT, sel_listner_button_move_left, 1);

			createArrowButton(g_selected, SWT.DOWN, sel_listener_move_down, 2);

			text_API_description = new Text(g_selected, SWT.WRAP | SWT.MULTI | SWT.BORDER | SWT.V_SCROLL);

			gd = new GridData(GridData.FILL_BOTH);
			gd.horizontalSpan = 2;
			gd.horizontalAlignment = SWT.FILL;
			gd.grabExcessHorizontalSpace = true;

			text_API_description.setLayoutData(gd);

			Group GParam = new Group(this, style);
			gd = new GridData(GridData.FILL_BOTH);
			gd.horizontalSpan = 4;
			gd.horizontalAlignment = SWT.FILL;
			gd.grabExcessHorizontalSpace = true;
			GParam.setLayoutData(gd);

			gl = new GridLayout();
			gl.numColumns = 7;
			GParam.setLayout(gl);
			GParam.setText("API 파라미터");

			Label Label1 = new Label(GParam, SWT.CENTER);
			gd = new GridData(GridData.FILL_BOTH);
			gd.horizontalSpan = 20;
			gd.grabExcessHorizontalSpace = true;

			Label1.setLayoutData(gd);
			Label1.setText("Input Parameter");

			new Table_ParamIn(GParam);

			Label Label2 = new Label(GParam, SWT.CENTER);
			gd = new GridData(GridData.FILL_BOTH);
			gd.horizontalSpan = 20;
			gd.grabExcessHorizontalSpace = true;

			Label2.setLayoutData(gd);
			Label2.setText("Output Parameter");
			new Table_ParamOut(GParam);

		}
	}

	private class tabDependencies extends Composite {

		tabDependencies(Composite c, int style) {

			super(c, style);
			gl = new GridLayout();
			gl.numColumns = 3;
			this.setLayout(gl);

			gd = new GridData(GridData.FILL_HORIZONTAL);
			gd.horizontalSpan = 3;
			gd.grabExcessHorizontalSpace = true;
			new Table_Dependencies(this);

		}
	}

	public void createArrowButton(Composite parent, int alignment, SelectionListener selectionListener, int choice) {

		GridData gd = null;
		switch (choice) {
		case 1:
			gd = new GridData(GridData.VERTICAL_ALIGN_CENTER | GridData.FILL_VERTICAL | GridData.GRAB_VERTICAL);
			break;
		case 2:
			gd = new GridData();
			break;
		}

		gd.heightHint = 100;
		final Button arwBtn = new Button(parent, SWT.ARROW);
		arwBtn.setAlignment(alignment);
		arwBtn.addSelectionListener(selectionListener);
		arwBtn.setLayoutData(gd);

	}

	SelectionListener sel_listener_move_up = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TableViewer viewer = Global.wizard_page2.tableviewer_selected_api;
			int index = viewer.getTable().getSelectionIndex();
			if (index == -1)
				return;

			List<Model_Base> table_list = ((Model_Service) viewer.getInput()).api_list;
			if (index != 0) {

				if (table_list.get(index).priority < table_list.get(index - 1).priority) {

					MessageDialog.openError(container.getShell(), "Warning", "API" + table_list.get(index).name + "은 "
							+ table_list.get(index - 1) + "보다 먼저 호출될 수 없습니다. " + "\n 우선순위 (priority)를 확인하세요.");
					return;
				}

				table_list.get(index).order -= 1;
				table_list.get(index - 1).order += 1;
				Collections.swap(table_list, index - 1, index);
				viewer.refresh();
				viewer.setSelection(new StructuredSelection(table_list.get(index - 1)));
				viewer.getTable().showSelection();
			}
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};

	SelectionListener sel_listener_move_down = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TableViewer viewer = Global.wizard_page2.tableviewer_selected_api;
			int index = viewer.getTable().getSelectionIndex();
			if (index == -1)
				return;

			List<Model_Base> table_list = ((Model_Service) viewer.getInput()).api_list;

			if (index < table_list.size() - 1) {

				if (table_list.get(index).priority > table_list.get(index + 1).priority) {

					MessageDialog.openError(container.getShell(), "Warning", "API" + table_list.get(index).name + "은 "
							+ table_list.get(index + 1) + "보다 늦게 호출될 수 없습니다." + "\n 우선순위 (priority)를 확인하세요.");
					return;
				}

				table_list.get(index).order += 1;
				table_list.get(index + 1).order -= 1;
				Collections.swap(table_list, index + 1, index);
				viewer.refresh();
				viewer.setSelection(new StructuredSelection(table_list.get(index + 1)));
				viewer.getTable().showSelection();
			}
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};

	SelectionListener sel_listner_button_move_left = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TableViewer viewer = Global.wizard_page2.tableviewer_selected_api;
			int index = viewer.getTable().getSelectionIndex();
			if (index == -1)
				return;

			List<Model_Base> table_list = ((Model_Service) viewer.getInput()).api_list;

			if (table_list.get(index).isDefaultParam) {
				MessageDialog.openError(container.getShell(), "Warning", "서비스 디폴트 API는 제거할 수 없습니다.");
				return;
			}

			table_list.remove(index);
			for (int i = 0; i < table_list.size(); i++) {
				table_list.get(i).order = i + 1;
			}

			Global.wizard_page2.tableviewer_selected_api
					.setSelection(new StructuredSelection(table_list.get(index - 1)));
			viewer.refresh();

			Global.isDependClear = true;

			ArrayList<String> duplicate_check = new ArrayList<String>();
			ArrayList<Model_Base> list_depend = new ArrayList<Model_Base>();
			for (Model_Base model : table_list) {

				String parentName = model.parent.name;
				if (duplicate_check.contains(parentName) == false) {
					duplicate_check.add(parentName);
					list_depend.add(model.parent);
					model.parent.get_dpendencies();
				}

			}

			Model_Service model_service = (Model_Service) Global.wizard_page2.tableviwer_dependencies.getInput();

			model_service.depend_list = list_depend;
			Global.wizard_page2.tableviwer_dependencies.refresh();

			Global.validate_service(true);

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};
	SelectionListener sel_listener_button_move_right = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			TreeViewer viewer = Global.wizard_page2.treeviewer_availabe_api;

			IStructuredSelection thisSelection = (IStructuredSelection) viewer.getSelection();
			if (thisSelection == null)
				return;
			Object selectedNode = thisSelection.getFirstElement();
			Model_Base model_old = (Model_Base) selectedNode;
			Model_Base model = new Model_Base(model_old);

			if (model.isAPI) {

				Model_Service ms = (Model_Service) Global.wizard_page2.tableviewer_selected_api.getInput();
				List<Model_Base> list = ms.api_list;

				ArrayList<Model_Base> new_list = new ArrayList<Model_Base>();

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

				Global.wizard_page2.tableviewer_selected_api.setInput(ms);

				Global.wizard_page2.tableviewer_selected_api.setSelection(new StructuredSelection(model), true);
				Global.validate_service(true);

				Global.isDependClear = true;

				Model_Service model_service = (Model_Service) Global.wizard_page2.tableviwer_dependencies.getInput();

				for (Model_Base m : model_service.depend_list) {
					if (model.parent.name.equals(m.name)) {
						return;
					}
				}
				model_service.depend_list.add(model.parent);
				model.parent.get_dpendencies();
				Global.wizard_page2.tableviwer_dependencies.setInput(model_service);

				Integer last_index = Global.wizard_page2.tableviewer_selected_api.getTable().getItemCount() - 1;

				Global.wizard_page2.tableviewer_selected_api.setSelection(
						new StructuredSelection(Global.wizard_page2.tableviewer_selected_api.getElementAt(last_index)),
						true);

			}

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}

	};

	public void updateStatus(String message) {

		setErrorMessage(message);
		if (Global.isDependClear == false) {
			setErrorMessage("찾을 수 없는 의존정보를 확인하세요");
		}
		setPageComplete(message == null && Global.isDependClear);

	}

}
