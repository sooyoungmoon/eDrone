
package org.etri.eDroneWizard;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;



import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IProjectDescription;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.Path;
import org.eclipse.core.runtime.jobs.IJobManager;
import org.eclipse.core.runtime.jobs.Job;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.operation.IRunnableWithProgress;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.wizard.Wizard;
import org.eclipse.ui.IEditorPart;
import org.eclipse.ui.INewWizard;
import org.eclipse.ui.IPerspectiveDescriptor;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchPage;
import org.eclipse.ui.IWorkbenchWindow;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.console.ConsolePlugin;
import org.eclipse.ui.console.IConsole;
import org.eclipse.ui.console.IConsoleManager;
import org.eclipse.ui.console.MessageConsole;
import org.eclipse.ui.console.MessageConsoleStream;
import org.eclipse.ui.ide.IDE;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;
import org.etri.eDroneModel.Model_Type;
import org.etri.eDroneModel.ParamType;

/**
 * @Class WizardMain
 * @brief INewWizard의 구현으로 Wizard 고유 기능을 지원한다.
 * @author etri
 * @version 1.0.0
 * @see page를 추가하거나 다음 page로 넘어갈 때 행위를 정의할 수 있다. Finish 버튼을 눌렀을 때 소스를 생성하고 빌드하는
 *      모든 작업이 들어있다.
 * 
 */

public class WizardMain extends Wizard implements INewWizard {

	private WizardPage1 page1;
	private WizardPage2 page2;
	private ISelection selection;

	public int index_argv;
	@SuppressWarnings("unused")
	private static int argCount;
	private static String projectRootPath = "";
	private static String projectName = "";
	private static String projectDescription = "";

	private static List<Model_Base> final_api_list = null;
	private static List<Model_Base> List_Param_Argument_Format = null;
	private static List<String> LCmpareParent;
	private static ArrayList<String> slSection = null;

	private static HashMap<String, HashMap<String, ArrayList<String>>> hmap_Sections = new HashMap<String, HashMap<String, ArrayList<String>>>();
	private static HashMap<String, HashMap<String, String>> RuleBook = new HashMap<String, HashMap<String, String>>();
	private static HashMap<String, String> hashmap_user_entered = new HashMap<String, String>();
	private static HashMap<String, HashMap<String, String>> hmap_Arg = new HashMap<String, HashMap<String, String>>();
	private static HashMap<String, String> rules = new HashMap<String, String>();
	private static HashMap<String, TData> store = new HashMap<String, TData>();
	private static HashMap<String, String> param_hash = new HashMap<String, String>();

	private static HashMap<String, Integer> api_call_count;
	private static ArrayList<String> many_time_callled_apis;
	private static ArrayList<String> must_call_onetime_list;
	private static ArrayList<String> called_list;
	private static HashMap<String, Integer> conver_count;
	private static File[] fileList;
	private static File fSection;

	public WizardMain() {
		super();
		setNeedsProgressMonitor(true);
	}

	@Override
	public void init(IWorkbench workbench, IStructuredSelection selection) {
		this.selection = selection;
	}

	/**
	 * 
	 * @brief page 인스턴스 생성 및 추가
	 * 
	 * @see 전역변수로도 등록한다.
	 * 
	 */

	@Override
	public void addPages() {

		page1 = new WizardPage1(selection);
		addPage(page1);
		Global.wizard_page1 = page1;

		page2 = new WizardPage2(selection);
		addPage(page2);
		Global.wizard_page2 = page2;

	}

	private MessageConsole findConsole(String name) {
		ConsolePlugin plugin = ConsolePlugin.getDefault();
		IConsoleManager conMan = plugin.getConsoleManager();
		IConsole[] existing = conMan.getConsoles();
		for (int i = 0; i < existing.length; i++) {
			if (name.equals(existing[i].getName()))
				return (MessageConsole) existing[i];
		}
		// no console found, so create a new one
		MessageConsole myConsole = new MessageConsole(name, null);
		conMan.addConsoles(new IConsole[] { myConsole });
		return myConsole;
	}

	/**
	 * @brief 위저드 Finish가 눌렸을 때 실행되는 함수
	 * 
	 * @see 춢력을 메시지 콘솔로 돌리고 doFinish()를 호출한다.
	 */

	@Override
	public boolean performFinish() {

		// 매개변수의 인덱스로 사용된다. 매번 Finish 버튼을 누를 때마 1로 초기화된다.
		// 0이 아닌 이유는 argv[1] 부터 매개변수이기 때문이다.
		index_argv = 1;

		// 메시지 콘솔 생성 및 스트림 전환
		// 이 작업이 이루어지지 않으면 유저가 사용중인 콘솔 창에서 표준 출력을 확인할 수 없다.
		// 그렇게되면 코드 생성 및 빌드 과정에서의 에러를 유저가 볼 수 없게 되므로 중요한 부분이다.
		MessageConsole newConsole = findConsole("Console");
		ConsolePlugin.getDefault().getConsoleManager().addConsoles(new IConsole[] { newConsole });
		MessageConsoleStream stream = newConsole.newMessageStream();
		PrintStream newStream = new PrintStream(stream);
		ByteArrayOutputStream os = new ByteArrayOutputStream();
		PrintStream myByteArrayOutputStream = new PrintStream(os);
		System.setOut(newStream); // link standard output stream to the console
		System.setErr(myByteArrayOutputStream); // link error output stream to the console

		// 첫번째페이지에서 프로젝트 생성 위치, 이름, 설명을 가져온다.
		projectName = page1.getProjectName();
		projectRootPath = page1.getDirName() + "/" + projectName;
		projectDescription = page1.text_ProjectDescription.getText();

		// 코드 생성시 프로젝트 이름을 활용하기 위해 해시로 집어넣는다.
		hashmap_user_entered.put("projectName", projectName);

		IRunnableWithProgress op = new IRunnableWithProgress() {
			public void run(IProgressMonitor monitor) throws InvocationTargetException {
				try {
					doFinish(monitor);

				} catch (CoreException e) {
					throw new InvocationTargetException(e);
				} finally {
					monitor.done();
				}
			}
		};
		try {
			getContainer().run(true, false, op);
		} catch (InterruptedException e) {
			return false;
		} catch (InvocationTargetException e) {
			Throwable realException = e.getTargetException();
			MessageDialog.openError(getShell(), "Error", realException.getMessage());
			return false;
		}
		MessageDialog.openInformation(getShell(), "eDrone", projectName + " 생성완료");

		IJobManager ij = Job.getJobManager();
		Job j = ij.currentJob();
		System.out.println(j);

		return true;
	}

	/**
	 * @brief 위저드 Finish가 눌렸을 때 실질적인 작업들이 들어있는 함수
	 * 
	 *        사용자가 입력한 정보는 결국 Model_API 클래스로 이루어진 프로젝트, API, 파라미터 정보이며 \n 이 함수에서 해당
	 *        정보에 기반하여 실제 소스 파일을 생성한다.\n 따라서 핵심이 되는 변수는 SharedVariabls 클래스에서 가져오는
	 *        service 리스트가 될 것이다.\n 이렇게 불러온다. List <Model_API> service =
	 *        mmlist.getList();\n
	 * 
	 *        파일별로 별도의 함수로 만들지 않았고 이 함수에 모두 뭉뚱그려져 있다.\n 각 파일별 들어가야할 내용이 하드코딩 되어있으며,
	 *        사용자 입력에 따라 변해야하는 부분만 변수 처리되어있다.\n 생성 순서는 다음과 같다.\n
	 * 
	 *        1. params.h\n 2. c++ 소스\n 3. .launch 파일\n 4. package.xml\n
	 * 
	 *        앞선 파일을 생성하면서 가공하는 정보를 뒤에서 사용하는 경우도 있으니 위의 순서를 굳이 변경하지 않는 것을 추천한다.\n
	 * 
	 * 
	 */

	private void doFinish(IProgressMonitor monitor) throws CoreException {

		monitor.beginTask("생성 시작 ... " + projectName, 7);
		try {
			Thread.sleep(500);
			monitor.worked(1);
			monitor.setTaskName("생성 ... params.h");
			Thread.sleep(500);

			monitor.worked(1);
			monitor.setTaskName("생성 ... " + projectName + ".launch");
			Thread.sleep(500);

			monitor.worked(1);
			monitor.setTaskName("생성 ... " + "ex_" + projectName + ".cpp");
			Thread.sleep(500);

			monitor.worked(1);
			monitor.setTaskName("생성 ... " + projectName + ".xml");
			Thread.sleep(500);

		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		api_call_count = new HashMap<String, Integer>();
		many_time_callled_apis = new ArrayList<String>();
		must_call_onetime_list = new ArrayList<String>();
		called_list = new ArrayList<String>();
		conver_count = new HashMap<String, Integer>();
		argCount = 1;
		mkdirs(projectRootPath);
		mkdirs(projectRootPath + "/inc");
		mkdirs(projectRootPath + "/inc/" + projectName);
		mkdirs(projectRootPath + "/launch");
		mkdirs(projectRootPath + "/src");

		List_Param_Argument_Format = new ArrayList<Model_Base>();

		Model_Service model_Service = (Model_Service) Global.wizard_page2.tableviewer_selected_api.getInput();
		final_api_list = model_Service.api_list;

		for (Model_Base s : final_api_list) {
			if (api_call_count.containsKey(s.name) == false) {
				api_call_count.put(s.name, 1);
			} else {
				many_time_callled_apis.add(s.name);
			}

			for (Model_Base p : s.child) {
				if (p.isArg) {
					s.arg_param_list.add(p);
				}

			}

		}

		LCmpareParent = new ArrayList<>();
		LCmpareParent.add("roscpp");
		LCmpareParent.add("rospy");
		LCmpareParent.add("std_msgs");
		for (Model_Base p : final_api_list) {

			String pName = p.getParent().toString();
			if (!LCmpareParent.contains(pName)) {

				LCmpareParent.add(pName);
			}

		}
		store.put("projectName", new TData(projectName));
		store.put("dependencies", new TData(LCmpareParent));
		store.put("description", new TData(projectDescription));

		try {
			createParamsh();
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			createSource();
		} catch (IOException e) {
			e.printStackTrace();
		}

		createCMakefile();
		rules = new HashMap<String, String>();
		createLaunchFile();
		createPackageXml();

		/////////////////////////////////////////////////////

		monitor.worked(1);
		monitor.setTaskName("빌드... " + projectName);

		/**
		 * @brief 파일을 모두 생성한 상태에서 Catkin 빌드를 실행한다.
		 * 
		 */

//		System.out.println("\n\n [  catkin build " + projectName + "  ]\n\n");
//
//		String[] cmd = new String[3];
//		cmd[0] = "catkin"; // check version of installed python: python -V
//		cmd[1] = "build";
//		cmd[2] = projectName;
//
//		try {
//			runCmd(cmd);
//		} catch (IOException e1) {
//			e1.printStackTrace();
//			System.out.println("catkin build fail");
//
//		}

		/**
		 * @brief eclipse 프로젝트 파일을 만드는 스크립트 (eclipseForceCmake.sh) 를 실행한다.
		 * 
		 *        해당 스크립트는 설치시 Catkin Workspace에 위치시킨다.\n
		 * 
		 */

		System.out.println(
				"\n\n  [   catkin build " + projectName + " --force-cmake -G\"Eclipse CDT4 - Unix Makefiles  ]   \n\n");

		String[] cmd2 = { Global.DEPNEDING_FILES_PATH + "/scripts/eclipseForceCmake.sh", projectName };

		try {
			runCmd(cmd2);
		} catch (IOException e1) {
			e1.printStackTrace();
			System.out.println("catkin force CMake fail");

		}

		/**
		 * @brief 생성된 프로젝트 파일들을 옮기는 스크립트를 실행한다.
		 * 
		 *        해당 스크립트는 설치시 Catkin Workspace에 위치시킨다.\n
		 * 
		 */

		System.out.println("\n\n  [   moving .project file  ] \n\n");
		String[] cmd3 = { Global.DEPNEDING_FILES_PATH + "/scripts/generateProjectFile.sh" };

		try {
			runCmd(cmd3);
		} catch (IOException e1) {
			e1.printStackTrace();
			System.out.println("catkin force CMake fail");
		}

		/**
		 * @brief 빌드 및 이클립스 프로젝트 변환이 완료된 프로젝트를 이클립스로 Import한다.
		 * 
		 * 
		 */
		monitor.worked(1);
		monitor.setTaskName("Importing...");
		getShell().getDisplay().asyncExec(new Runnable() {
			public void run() {
				IWorkbenchWindow window = PlatformUI.getWorkbench().getActiveWorkbenchWindow();
				String perspectiveId = "org.eclipse.cdt.ui.CPerspective";
				IPerspectiveDescriptor desc = window.getWorkbench().getPerspectiveRegistry()
						.findPerspectiveWithId(perspectiveId);

				if (desc != null) {
					IWorkbenchPage activePage = window.getActivePage();
					IPerspectiveDescriptor[] arr = activePage.getOpenPerspectives();

					for (int i = 0; i < arr.length; i++) {
						if (arr[i].getId().equals(perspectiveId)) {
							activePage.closePerspective(arr[i], false, false);
						}
					}
					if (activePage != null) {
						activePage.setPerspective(desc);
					}
				}

			}
		});

		System.out.println("\n\n  [   importing project into eclipse   ] \n\n");
		importProject(projectName, monitor);

		getShell().getDisplay().asyncExec(new Runnable() {
			public void run() {

				IJobManager ij = Job.getJobManager();
				Job[] list = ij.find(null);

				for (Job j : list) {
					if (j.getName().equals("C/C++ Indexer")) {
						try {
							j.join();
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
				}
			}
		});

	}

	/*
	 * [project root]/inc/<projectName>/params.h 를 생성한다.
	 */

	public void createParamsh() throws IOException {

		File FParam = mkfiles(projectRootPath + "/inc/" + projectName + "/params.h");

		FileWriter fw;
		StringBuilder sb = new StringBuilder("\n\n");

		try {
			fw = new FileWriter(FParam, true);

			for (Model_Base s : final_api_list) {

				List<Model_Base> param = s.child;

				for (Model_Base p : param) {

					String value = p.getValue();
					if (p.isArg == false) { // 매개변수가 아니면 params.h에 등록을 해야한다.

						if (p.isIn) {

							String name = p.toString();
							// params.xml에 type 변형에 대한 정보가 있다.
							String type = p.getType();
							if (Global.hashmap_type.containsKey(type)) {
								Model_Type mt = Global.hashmap_type.get(type);
								type = mt.beConverted;
							} else {

								if (type.indexOf("/") != -1) {
									String tokens[] = type.split("/");
									type = tokens[tokens.length - 1];
								}

								if (Global.hashmap_headers.containsKey(type + ".h")) {

									String abosolute_path = Global.hashmap_headers.get(type + ".h").get(0);
									String[] tokens = abosolute_path.split("/include/");

									String header = "#include <" + tokens[1] + ">\n";
									sb.insert(0, header);
								}
							}

							if (p.ptype.equals(ParamType.Text) || p.ptype.equals(ParamType.Combo)) {

								String tokens[] = value.split(";");
								value = tokens[tokens.length - 1];

								try {
									Integer.parseInt(value);
								} catch (Exception e8x) {
									value = "\"" + value + "\"";
								}
								sb.append("const " + type + " " + name.toUpperCase() + " = " + value + ";\n\n");
								continue;

							} else if (p.ptype.equals(ParamType.Vector)) {

								type = "std::string";

								String tokens[] = value.split(";");
								value = tokens[tokens.length - 1];
								if (value.equals("None")) {
									System.out.println("none occured");
								}

							} else if (p.ptype.equals(ParamType.Class)) {

								type = "std::string";

								if (value.replace(" ", "").length() < 1)
									break;
								String tokens[] = value.split(";");

								StringBuilder sb2 = new StringBuilder();
								if (tokens.length < 1) {
									value = "";
									break;
								}
								int i = 1;
								for (; i < tokens.length - 1; i++) {
									String ts[] = tokens[i].split(",");
									String target = ts[2];
									if (target.length() < 1) {
										System.out.println("length <1");
									}
									sb2.append(target);
									sb2.append(",");
								}
								String ts[] = tokens[i].split(",");
								String target = ts[2];
								if (target.length() < 1) {
									System.out.println("length <1");
								}
								sb2.append(target);
								value = sb2.toString();

							}
							String variable_name = name.toUpperCase();
							if (many_time_callled_apis.contains(s.name) == true) {

								Integer count = api_call_count.get(s.name);
								if (count != 1) {
									variable_name = variable_name + count.toString();
								}

								api_call_count.put(s.name, count + 1);

							}

							try {
								Integer.parseInt(value);
							} catch (Exception e8x) {
								value = "\"" + value + "\"";
							}

							sb.append("const " + type + " " + variable_name + " = " + value + ";\n\n");
						}

					} else { // 매개변수이면
						if (p.isIn) {

							List_Param_Argument_Format.add(p);

						}
					}
				}
			}

			System.out.println(sb.toString());

			fw.write(sb.toString());
			fw.flush();
			fw.close();

		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private void createSource() throws FileNotFoundException, IOException {

		File newFile = mkfiles(projectRootPath + "/src/ex_" + projectName + ".cpp");

		// section.tpl 리스트
		slSection = new ArrayList<String>();

		FilenameFilter filter = new FilenameFilter() {

			@Override
			public boolean accept(File dir, String name) {

				if (name.endsWith(".tpl")) {
					return true;
				}

				return false;
			}
		};

		// 파일 객체 생성
		String templatePath = Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/template/source";
		File dir = new File(templatePath);
		fileList = dir.listFiles(filter);

		for (File file : fileList) {

			if (file.isFile() && file.getName().startsWith(".") == false) {

				System.out.println("\n>> " + file.getName());
				String tplName = file.getName().substring(0, file.getName().length() - 4);

				switch (tplName) {
				case "section":
					fSection = file;
					FileReader filereader = new FileReader(fSection);
					// 입력 버퍼 생성
					BufferedReader bufReader = new BufferedReader(filereader);
					String line = "";
					while ((line = bufReader.readLine()) != null) {
						slSection.add(line);
					}
					bufReader.close();

				default:
					FileReader filereader2 = new FileReader(file);
					BufferedReader bufReader2 = new BufferedReader(filereader2);
					boolean isSectionOpen = false;
					// Rule HashMap 추가
					HashMap<String, String> hmap_rule = new HashMap<String, String>();

					String line2 = "";
					String secName = "";
					ArrayList<String> list = new ArrayList<>();
					while ((line2 = bufReader2.readLine()) != null) {

						System.out.println(line2);

						if (line2.indexOf("---") != -1) {

							if (isSectionOpen == true) {
								isSectionOpen = false;

								if (hmap_Sections.containsKey(secName)) {
									HashMap<String, ArrayList<String>> temp = hmap_Sections.get(secName);
									temp.put(tplName, list);
								} else {
									HashMap<String, ArrayList<String>> map = new HashMap<String, ArrayList<String>>();
									map.put(tplName, list);
									hmap_Sections.put(secName, map);
								}
								continue;
							} else if (isSectionOpen == false) {
								isSectionOpen = true;
								line2 = line2.trim();

								if (line2.indexOf(",") != -1) {

									String tokens[] = line2.split(",");
									line2 = tokens[0];
									secName = line2.substring(3, line2.length());
									if (Integer.parseInt(tokens[1]) == 1) {

										String idx = tplName + secName;

										if (must_call_onetime_list.contains(idx) == false) {
											must_call_onetime_list.add(idx);
										}
									}
									list = new ArrayList<String>();
									continue;
								}

								secName = line2.substring(3, line2.length());
								list = new ArrayList<String>();
								System.out.println(tplName + " -> " + secName);
								continue;
							}
						}

						if (isSectionOpen == true) {

							list.add(line2);

						} else {
							if (line2.indexOf(":") != -1) {
								String[] tokens = line2.split(":");
								hmap_rule.put("?[" + tokens[0] + "]", tokens[1]);
							}
						}
					}

					if (hmap_rule.size() != 0) {
						RuleBook.put(tplName, hmap_rule);
					}
					bufReader2.close();
				}
			}
		}

		if (fSection == null) {
			System.out.println("Cannot find section.tpl!!");
			return;
		}

		BufferedWriter bufferedWriter = null;
		try {
			// 파일 객체 생성
			// File newFile = new File("/home/neoweb/test.txt");
			bufferedWriter = new BufferedWriter(new FileWriter(newFile));

			if (newFile.isFile() && newFile.canWrite()) {
				// 쓰기

				for (int i = 0; i < slSection.size(); i++) {

					String string = slSection.get(i);

					if (string.indexOf("--") != -1) {

						String secName = string.substring(2, string.length());
						secName = secName.trim();
						if (hmap_Sections.containsKey(secName)) {
							System.out.println("Writing ... " + secName);

							HashMap<String, ArrayList<String>> map = hmap_Sections.get(secName);

							if (map.containsKey("Global")) {
								ArrayList<String> li = map.get("Global");
								Model_Base temp = new Model_Base("Global", null);
								write_by_rule(temp, bufferedWriter, li);
								map.remove("Global");
							}
							for (Model_Base model : final_api_list) {

								if (map.containsKey(model.name) == false)
									continue;

								String tplName = model.name;

								String idx = tplName + secName;
								ArrayList<String> li = map.get(tplName);
								if (must_call_onetime_list.contains(idx)) {

									if (called_list.contains(idx)) {

									} else {

										write_by_rule(model, bufferedWriter, li);
										called_list.add(idx);
									}
								} else {
									write_by_rule(model, bufferedWriter, li);
								}
							}
						}

					} else {
						System.out.println("E " + string);
						bufferedWriter.write(string);
						bufferedWriter.newLine();
					}
				}

				bufferedWriter.close();
			}
		} catch (IOException e) {
			System.out.println(e);
		} finally {
			if (bufferedWriter != null)
				bufferedWriter.close();
		}

	}

	private void write_by_rule(Model_Base Model_API, BufferedWriter buf_writer, ArrayList<String> li) {

		HashMap<String, String> hmap_rule = RuleBook.get(Model_API.name);

		for (String line2 : li) {
			if (hmap_rule.size() != 0) {

				// 주석일 경우 그대로 담고 continue;
				if (line2.indexOf("//") != -1) {
					String temp3 = line2.trim();
					String tok = temp3.substring(0, 2);
					if (tok.equals("//")) {

						System.out.println(line2);
						try {

							buf_writer.write(line2);
							buf_writer.newLine();
						} catch (IOException e) {
							e.printStackTrace();
						}

						continue;
					}

				}

				// 대치되는 변수가 있을 경우 '?[ '존재 여부로 판단
				if (line2.indexOf("?[") != -1) {

					// 매개변수 지정함수인지 먼저 확인
					for (Model_Base p : Model_API.arg_param_list) {

						String find = "?[" + p.name.toUpperCase() + "]";
						System.out.println("~ " + find);
						if (line2.indexOf(find) != -1) {

							String type = p.type;

							if (param_hash.containsKey(type)) {
								type = param_hash.get(type);
							}

							if (Global.hashmap_type.containsKey(type)) {
								type = Global.hashmap_type.get(type).beConverted;
							}

							switch (type) {

							case "int":
							case "double":
							case "uint":
								line2 = line2.replace(find, "atof(argv[" + index_argv + "])");
								break;
							default:
								line2 = line2.replace(find, "argv[" + index_argv + "]");

							}
							index_argv++;
							continue;
						}

					}

					if (line2.indexOf("#ifndef ?[PRJ]") != -1) {
						System.out.println("test");

					}

					for (String key : hmap_rule.keySet()) {

						if (line2.indexOf(key) != -1) {

							if (hashmap_user_entered.containsKey(hmap_rule.get(key))) {
								line2 = line2.replace(key, hashmap_user_entered.get(hmap_rule.get(key)));
							} else {

								String replace = hmap_rule.get(key);
								if (conver_count.containsKey(replace)) {
									Integer count = conver_count.get(replace);
									line2 = line2.replace(key, hmap_rule.get(key) + count.toString());
									conver_count.put(replace, count + 1);
								} else {
									conver_count.put(replace, 2);
									line2 = line2.replace(key, hmap_rule.get(key));
								}
							}
						}
					}
				}
			}
			System.out.println(line2);
			try {
				buf_writer.write(line2);
				buf_writer.newLine();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	/*
	 * [project root]/launch/[project name].launch 를 생성한다.
	 */

	private void createLaunchFile() {

		File Flaunch = mkfiles(projectRootPath + "/launch/" + projectName + ".launch");

		FileWriter fw3;
		StringBuilder sb = new StringBuilder();

		try {

			fw3 = new FileWriter(Flaunch, true);

			boolean isTemplateEntered = false;

			File file = new File(
					Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/template/package/launch.tpl");
			FileReader reader = new FileReader(file);
			@SuppressWarnings("resource")
			BufferedReader bufReader = new BufferedReader(reader);

			String line = "";
			while ((line = bufReader.readLine()) != null) {
				if (line.indexOf("=======") != -1) {
					isTemplateEntered = true;
					continue;
				}

				if (isTemplateEntered == false) {

					if (line.indexOf("#") != -1)
						continue;

					line = line.replace(" ", "");
					String[] tokens = line.split(":");
					if (tokens == null || tokens.length < 2)
						continue;

					String from = tokens[0];
					String to = tokens[1];

					rules.put(from, to);

				} else {
					if (line.length() < 1) {
						sb.append("\n");
						continue;
					}

					if (line.indexOf("?[") != -1) {

						for (String key : rules.keySet()) {
							String from = "?[" + key + "]";
							if (line.indexOf(from) != -1) {

								TData data = store.get(rules.get(key));
								if (data.isList) {
									for (String item : data.getList()) {
										sb.append("\t" + item + "\n");
									}

								} else {
									line = line.replace(from, data.getStr());
									sb.append(line + "\n");
								}
							}
						}
						continue;
					}

					// System.out.println(line);
					sb.append(line + "\n");
				}
			}

			fw3.write(sb.toString());
			fw3.flush();
			fw3.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	/*
	 * [project root]/package.xml 을 생성한다.
	 */

	private void createPackageXml() {

		File FPackage = mkfiles(projectRootPath + "/package.xml");

		FileWriter fw4;
		StringBuilder sb = new StringBuilder();

		try {
			fw4 = new FileWriter(FPackage, true);

			
			boolean isTemplateEntered = false;

			File file = new File(
					Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/template/package/package.tpl");
			FileReader reader = new FileReader(file);
			@SuppressWarnings("resource")
			BufferedReader bufReader = new BufferedReader(reader);

			String line = "";
			while ((line = bufReader.readLine()) != null) {
				if (line.indexOf("=======") != -1) {
					isTemplateEntered = true;
					continue;
				}

				if (isTemplateEntered == false) {

					if (line.indexOf("#") != -1)
						continue;

					line = line.replace(" ", "");
					String[] tokens = line.split(":");
					if (tokens == null || tokens.length < 2)
						continue;

					String from = tokens[0];
					String to = tokens[1];

					rules.put(from, to);

				} else {
					if (line.length() < 1) {
						sb.append("\n");
						continue;
					}
					
					
					if (line.indexOf("?[") != -1) {

						for (String key : rules.keySet()) {
							String from = "?[" + key + "]";
							if (line.indexOf(from) != -1) {

								TData data = store.get(rules.get(key));
								if (data.isList) {
									for (String item : data.getList()) {
										sb.append("\t<depend>" + item + "</depend>\n");
									}

								} else {
									line = line.replace(from, data.getStr());
									sb.append(line + "\n");
								}
							}
						}
						continue;
					}

					sb.append(line + "\n");
				}
				
			}
//			sb.append("<?xml version=\"1.0\"?>\n\n" + "<package format=\"2\">\n" + "\t<name>" + projectName
//				
//					+ "</name>\n" + "\n" + "\t<version> 0.0.1 </version>\n" + "\n" + "\t<description>"
//					+ projectDescription + "</description>\n" + "\n"
//					+ "\t<maintainer email=\"sample@gmail.com\">sample.maintainter</maintainer>\n" + "\n"
//					+ "\t<license>TODO</license>\n" + "\n" + "\t<buildtool_depend>catkin</buildtool_depend>\n\n");
//
//			for (String pName : LCmpareParent) {
//				sb.append("\t<depend>" + pName + "</depend>\n");
//
//			}

//			sb.append("\n</package>");

			fw4.write(sb.toString());
			fw4.flush();
			fw4.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private void createCMakefile() {

		/*
		 * [project root]/CMakeList.txt 를 생성한다.
		 */

		File FCmake = mkfiles(projectRootPath + "/CMakeLists.txt");

		FileWriter fw5;
		StringBuilder sb = new StringBuilder();
		try {

			fw5 = new FileWriter(FCmake, true);

			boolean isTemplateEntered = false;

			File file = new File(
					Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/template/package/cmakelist.tpl");
			FileReader reader = new FileReader(file);
			@SuppressWarnings("resource")
			BufferedReader bufReader = new BufferedReader(reader);

			String line = "";
			while ((line = bufReader.readLine()) != null) {

				if (line.indexOf("=======") != -1) {
					isTemplateEntered = true;
					continue;
				}

				if (isTemplateEntered == false) {

					if (line.indexOf("#") != -1)
						continue;

					line = line.replace(" ", "");
					String[] tokens = line.split(":");
					if (tokens == null || tokens.length < 2)
						continue;

					String from = tokens[0];
					String to = tokens[1];

					rules.put(from, to);

				} else {
					if (line.length() < 1) {
						sb.append("\n");
						continue;
					}

					if (line.indexOf("?[") != -1) {

						for (String key : rules.keySet()) {
							String from = "?[" + key + "]";
							if (line.indexOf(from) != -1) {

								TData data = store.get(rules.get(key));
								if (data.isList) {
									for (String item : data.getList()) {
										// System.out.println("\t" + item);
										sb.append("\t" + item + "\n");
									}

								} else {
									line = line.replace(from, data.getStr());
									sb.append(line + "\n");
									// System.out.println(line);
								}
							}
						}
						continue;
					}

					sb.append(line + "\n");
				}
			}

			fw5.write(sb.toString());
			fw5.flush();
			fw5.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private static void runCmd(String[] cmd) throws IOException {

		Process pb = Runtime.getRuntime().exec(cmd, null, new File(Global.CATKIN_WORKSPACE_PATH));

		String line;
		BufferedReader input = new BufferedReader(new InputStreamReader(pb.getInputStream()));
		while ((line = input.readLine()) != null) {
			System.out.println(line);
		}
		input.close();
	}

	/**
	 * @brief 생성된 프로젝트를 가져오고 ex_<프로젝트 이름>.cpp를 사용자가 볼 수 있도록 에디터로 오픈한다.
	 * 
	 */

	private void importProject(String projectName, IProgressMonitor monitor) throws CoreException {

		String buildPath = Global.CATKIN_WORKSPACE_PATH + "build/" + projectName;

		File fi = new File(buildPath);
		if (!fi.exists()) {
			System.out.print("Catkin build or force Cmake was not successful");
			return;
		}

		System.out.println(".project file path = " + buildPath + "/.project");
		IProjectDescription description2 = ResourcesPlugin.getWorkspace()
				.loadProjectDescription(new Path(buildPath + "/.project"));

		try {
			IProject project = ResourcesPlugin.getWorkspace().getRoot().getProject(description2.getName());
			project.create(description2, null);
			project.open(null);

		} catch (CoreException e) {
			System.out.println(e);
			return;
		}

		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IResource resource = root.findMember(new Path(projectName + "@" + projectName));

		monitor.worked(1);
		monitor.setTaskName("프로젝트 임포트 완료. 인덱스 리빌드 ... ");

		IContainer container = (IContainer) resource;
		
		final IFile file = container.getFile(new Path("[Source directory]/src/ex_" + projectName + ".cpp"));
		getShell().getDisplay().asyncExec(new Runnable() {
			public void run() {
				IWorkbenchPage page = PlatformUI.getWorkbench().getActiveWorkbenchWindow().getActivePage();
				try {
					IDE.openEditor(page, file, true);

				} catch (PartInitException e) {
				}
			}
		});
	}

	/**
	 * @brief 디렉토리 생성 함수
	 * 
	 */
	private void mkdirs(String path) {
		File desti = new File(path);

		// 해당 디렉토리의 존재여부를 확인

		if (!desti.exists()) {

			// 없다면 생성

			desti.mkdirs();

		} else {

			// 있다면 현재 디렉토리 파일을 삭제

			File[] destroy = desti.listFiles();

			for (File des : destroy) {

				des.delete();

			}

		}

	}

	/**
	 * @brief 파일 생성함수
	 * 
	 */
	private File mkfiles(String path) {

		// 해당 디렉토리의 존재여부를 확인
		File file = null;
		try {

			// 파일 객체 생성
			file = new File(path);

		} catch (Exception e) {
			e.printStackTrace();
		}

		return file;

	}

	private static class TData {

		private boolean isList;
		private List<String> list;
		private String str;

		public TData(List<String> param) {
			isList = true;
			list = param;
		}

		public TData(String param) {
			str = param;
		}

		public List<String> getList() {
			return list;
		}

		public String getStr() {
			return str;
		}
	}
}
