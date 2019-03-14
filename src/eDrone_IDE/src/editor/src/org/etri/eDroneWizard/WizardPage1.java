package org.etri.eDroneWizard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.wizard.IWizardPage;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.ModifyEvent;
import org.eclipse.swt.events.ModifyListener;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.layout.FormLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.DirectoryDialog;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;

/**
 * @Class WizardPage1
 * @brief 시나리오 선택 및 기본 정보 입력 페이지 구현 클래스
 * 
 * 
 */

public class WizardPage1 extends WizardPage implements IWizardPage {

	private ISelection selection;

	// 서비스 현황 트리
	public TreeViewer treeviewer_service;

	// MyModel은 API와 프로젝트 정보가 저장되는 객체
	// 트리, 테이블, 리스트가 이 객체의 정보를 참조하여 보여지게됨

	// 환경변수로 등록되어있는 CATKIN_WS_PATH를 읽어와 서 저장.

	// GUI에서 GridLayout 형식으로 배치하기 위해서 사용되는 변수들
	private GridData gd;
	private GridLayout gl;

	// 텍스트 객체
	private Text text_ProjectName; // 프로젝트 이름
	private Text text_ProjectLocation; // 생성 프로젝트 경로
	// 라벨 객체 (생성 후 내용이 바뀌지 않는 것들이라 중복으로 사용)
	private Label label;

	// 디렉토리 탐색 버튼
	private Button bBrowse;

	public Text text_ServiceDescription;
	public Text text_Warning;
	public Text text_ProjectDescription;
	public Composite composite_title_03;
	public Composite composite_title_04;

	public boolean isAPIClear;

	@SuppressWarnings("unused")
	private Composite_Services composite_services = null;

	public WizardPage1(ISelection selection) {
		super("wizardPage");

		// 위저드 제목
		setTitle("eDrone Project");
		setImageDescriptor(Global.imagedescriptor_Logo_2x);
		// 위저드 제목 아래에 표시되는 메세지
		setDescription("eDrone 프로젝트를 시작합니다.");
		this.selection = selection;
	}

	// 이곳에 UI가 배치된다.

	@Override
	public IWizardPage getNextPage() {
		set_available_api_tree();
		load_service();
		return Global.wizard_page2;
	}

	@Override
	public void createControl(Composite parent) {

		local_initialize();

		final Shell shell = getShell();
		shell.setSize(500, 900); // 위저드 창 크기는 여기서 조절할 수 있다.

		Composite container = new Composite(parent, SWT.V_SCROLL);
		gl = new GridLayout();
		gl.numColumns = 4; // numColumns 는 위저드 화면을 세로로 몇등분할지를 정한다.
		gl.marginLeft = 30;
		gl.marginRight = 30;
		gl.verticalSpacing = 10;
		gl.makeColumnsEqualWidth = true;
		gl.marginTop = 20;
		gl.marginBottom = 20;

		container.setLayout(gl);

		label = new Label(container, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 1; // 3칸 중 한칸을 차지하는 라벨을 만들겠다는 의미
		label.setLayoutData(gd);
		label.setText("&Project Name :");

		text_ProjectName = new Text(container, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		text_ProjectName.setLayoutData(gd);
		text_ProjectName.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				dialogChanged();
			}
		});

		label = new Label(container, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 1;
		label.setLayoutData(gd);

		label = new Label(container, SWT.NULL);
		gd = new GridData();
		gd.horizontalSpan = 1;
		label.setLayoutData(gd);
		label.setText("&Location :");

		text_ProjectLocation = new Text(container, SWT.BORDER | SWT.SINGLE);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		text_ProjectLocation.setLayoutData(gd);

		// Linux
		text_ProjectLocation.setText(Global.CATKIN_WORKSPACE_PATH + "/src");
		text_ProjectLocation.setEditable(false);
		// /Linux

		text_ProjectLocation.addModifyListener(new ModifyListener() {
			@Override
			public void modifyText(ModifyEvent e) {
				dialogChanged();
			}
		});

		// 디렉토리 브라우징 버튼
		bBrowse = new Button(container, SWT.PUSH);
		bBrowse.setText("Browse...");
		bBrowse.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(SelectionEvent e) {
				handleBrowse();
			}
		});

		Composite composite_title_01 = new Composite(container, SWT.NULL);
		gd = new GridData(GridData.HORIZONTAL_ALIGN_FILL);
		gd.horizontalSpan = 2;
		gd.heightHint = 35;
		FormLayout layout = new FormLayout();
		composite_title_01.setLayoutData(gd);
		composite_title_01.setLayout(layout);

		Composite composite_title_02 = new Composite(container, SWT.NULL);
		gd = new GridData(GridData.HORIZONTAL_ALIGN_FILL);
		gd.horizontalSpan = 2;
		gd.heightHint = 35;
		layout = new FormLayout();
		composite_title_02.setLayoutData(gd);
		composite_title_02.setLayout(layout);

		Label label_image_01 = new Label(composite_title_01, SWT.NULL);
		FormData fd = new FormData(30, 30);
		fd.left = new FormAttachment(0, 0);
		fd.top = new FormAttachment(0, 10);
		label_image_01.setLayoutData(fd);
		label_image_01.setImage(Global.image_Tree);

		Label label_title_01 = new Label(composite_title_01, SWT.LEFT);
		fd = new FormData(100, 30);
		fd.left = new FormAttachment(0, 30);
		fd.top = new FormAttachment(0, 13);
		label_title_01.setLayoutData(fd);
		label_title_01.setText("서비스 트리");

		Label label_image_02 = new Label(composite_title_02, SWT.CENTER);
		fd = new FormData(30, 30);
		fd.left = new FormAttachment(0, 0);
		fd.top = new FormAttachment(0, 10);
		label_image_02.setLayoutData(fd);
		label_image_02.setImage(Global.image_List);

		Label label_title_02 = new Label(composite_title_02, SWT.LEFT);
		fd = new FormData(100, 30);
		fd.left = new FormAttachment(0, 30);
		fd.top = new FormAttachment(0, 13);
		label_title_02.setLayoutData(fd);
		label_title_02.setText("호출 API 리스트");

		composite_services = new Composite_Services(container);

		composite_title_03 = new Composite(container, SWT.NULL);
		gd = new GridData(GridData.HORIZONTAL_ALIGN_FILL);
		gd.horizontalSpan = 4;
		gd.minimumHeight = 35;
		gd.widthHint = 100;
		layout = new FormLayout();
		composite_title_03.setLayoutData(gd);
		composite_title_03.setLayout(layout);

		composite_title_04 = new Composite(container, SWT.NULL);
		gd = new GridData(GridData.HORIZONTAL_ALIGN_FILL);
		gd.horizontalSpan = 4;
		gd.minimumHeight = 35;
		gd.widthHint = 100;
		layout = new FormLayout();
		composite_title_04.setLayoutData(gd);
		composite_title_04.setLayout(layout);

		Label label_image_03 = new Label(composite_title_03, SWT.NULL);
		label_image_03.setImage(Global.image_info);

		fd = new FormData(30, 30);
		fd.left = new FormAttachment(0, 0);
		fd.top = new FormAttachment(0, 13);
		label_image_03.setLayoutData(fd);

		text_ServiceDescription = new Text(composite_title_03, SWT.WRAP | SWT.MULTI | SWT.V_SCROLL | SWT.BORDER);

		fd = new FormData();
		fd.left = new FormAttachment(0, 30);
		fd.top = new FormAttachment(0, 13);
		fd.width = 800;
		fd.height = 70;
		text_ServiceDescription.setLayoutData(fd);
		text_ServiceDescription.setEditable(false);

		Label label_image_04 = new Label(composite_title_04, SWT.NULL);
		label_image_04.setImage(Global.image_Warn);

		fd = new FormData(30, 30);
		fd.left = new FormAttachment(0, 0);
		fd.top = new FormAttachment(0, 13);
		label_image_04.setLayoutData(fd);

		text_Warning = new Text(composite_title_04, SWT.WRAP | SWT.MULTI | SWT.V_SCROLL | SWT.BORDER);

		fd = new FormData();
		fd.left = new FormAttachment(0, 30);
		fd.top = new FormAttachment(0, 13);
		fd.width = 800;
		fd.height = 70;
		text_Warning.setLayoutData(fd);
		text_Warning.setEditable(false);

		Label label = new Label(container, SWT.NULL);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 4;
		gd.horizontalAlignment = GridData.BEGINNING;
		label.setText("프로젝트 요약");

		text_ProjectDescription = new Text(container, SWT.BORDER | SWT.WRAP | SWT.MULTI | SWT.V_SCROLL);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.grabExcessHorizontalSpace = true;
		gd.heightHint = 100;
		gd.horizontalSpan = 4;
		text_ProjectDescription.setLayoutData(gd);
		text_ProjectDescription.setText("프로젝트 관련 사항을 자유롭게 기입해주세요.");

		// 프로젝트 이름이 입력되었는지, 중복된 이름의 프로젝트가 없는지 체크하는 함수
		dialogChanged();

		// 페이지가 위저드에 보여지기 위해 필수적으로 실행되어야하는 함수
		setControl(container);

	}

	public void set_available_api_tree() {
		TreeViewer tv = Global.wizard_page2.treeviewer_availabe_api;
		tv.setInput(Global.MODEL_API_ROOT);
		tv.expandAll();
	}

	@Override
	public boolean canFlipToNextPage() {
		return isPageComplete();
	}

	public void load_service() {

		if (Global.Service_Selected != null) {
			if (Global.Service_Selected.file_path.equals(Global.Service_Temp.file_path)) {
				return;
			}
		}

		Global.wizard_page2.tableviewer_selected_api.setInput(Global.Service_Temp);
		Global.Service_Selected = Global.Service_Temp;

		List<Model_Base> list = Global.Service_Selected.api_list;
		if (list.size() > 0) {
			Global.wizard_page2.tableviewer_selected_api.setSelection(new StructuredSelection(list.get(0)));
		}

		Global.isDependClear = true;
		ArrayList<String> duplicate_check = new ArrayList<String>();
		ArrayList<Model_Base> list_depend = new ArrayList<Model_Base>();
		for (Model_Base model : list) {

			String parentName = model.parent.name;
			if (duplicate_check.contains(parentName) == false) {
				duplicate_check.add(parentName);
				list_depend.add(model.parent);
			}

			model.parent.get_dpendencies();

		}
		Model_Service model_service = (Model_Service) Global.wizard_page2.tableviwer_dependencies.getInput();

		model_service.depend_list = list_depend;

		Global.wizard_page2.tableviwer_dependencies.setInput(model_service);

		Global.validate_service(true);

	}

	/**
	 * \fn listDependencies \brief 시나리오에서 필요하다고 되어있는 API가 실제로 존재하는지 체크하는 함수
	 */

	private void local_initialize() {

		isAPIClear = false;
		Global.initialize();
		setPageComplete(false);

	}

	public void set_warning_message(String message) {
		text_Warning.setText(message);
	}

	private void handleBrowse() {

		DirectoryDialog dirDialog = new DirectoryDialog(getShell());
		dirDialog.setText("Select directory");
		String dirPath = dirDialog.open();
		if (selection != null)
			text_ProjectLocation.setText(dirPath);

	}

	public void dialogChanged() {

		if (getProjectName().length() == 0) {
			updateStatus("프로젝트 이름을 입력해주세요");
			return;
		} else if (getProjectName().indexOf("_") != -1) {
			updateStatus("프로젝트 이름에 언더바는 포함될 수 없습니다.");
			return;
		}

		String fileName = getDirName() + "/" + getProjectName();
		File f = new File(fileName);

		if (f.exists()) {
			updateStatus("중복된 이름의 프로젝트가 있습니다");
		} else if (isAPIClear == false) {
			updateStatus("서비스를 선택하세요");
		} else {
			updateStatus(null);
		}
	}

	public void updateStatus(String message) {
		setErrorMessage(message);
		setPageComplete(message == null && isAPIClear);
	}

	public String getDirName() {
		return text_ProjectLocation.getText();
	}

	public String getProjectName() {
		return text_ProjectName.getText();
	}

}
