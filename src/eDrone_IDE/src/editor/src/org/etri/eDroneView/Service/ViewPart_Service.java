package org.etri.eDroneView.Service;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.eclipse.jface.resource.JFaceResources;
import org.eclipse.jface.resource.LocalResourceManager;
import org.eclipse.jface.resource.ResourceManager;
import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider.IStyledLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.StyledString;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.TreeViewerColumn;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerDropAdapter;
import org.eclipse.swt.SWT;
import org.eclipse.swt.dnd.DND;
import org.eclipse.swt.dnd.DragSourceEvent;
import org.eclipse.swt.dnd.DragSourceListener;
import org.eclipse.swt.dnd.DropTargetEvent;
import org.eclipse.swt.dnd.TextTransfer;
import org.eclipse.swt.dnd.Transfer;
import org.eclipse.swt.dnd.TransferData;
import org.eclipse.swt.events.KeyAdapter;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.RGBA;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.Text;
import org.eclipse.ui.dialogs.FilteredTree;
import org.eclipse.ui.dialogs.PatternFilter;
import org.eclipse.ui.part.ViewPart;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;
import org.etri.eDroneModel.ParamType;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class ViewPart_Service extends ViewPart {

	private ResourceManager resourceManager;

	public TreeViewer treeviewer_Service;
	private TableViewer tableviewer_selected_api;

	private DocumentBuilderFactory factory;
	private DocumentBuilder builder = null;
	private Document doc = null;

	private final int HEIGHT = 150;
	private final int WIDTH = 400;

	private Text text_description;

	public Model_Service model_Service = null;

	private File ServiceRootfile;

	public ViewPart_Service() {
		super();
	}

	@Override
	public void createPartControl(Composite parent) {

		GridLayout gl = new GridLayout();

		Composite container = new Composite(parent, SWT.V_SCROLL);
		gl = new GridLayout();
		gl.numColumns = 4; // numColumns 는 위저드 화면을 세로로 몇등분할지를 정한다.
		gl.marginLeft = 30;
		gl.marginRight = 30;
		gl.verticalSpacing = 10;
		gl.marginTop = 20;
		gl.marginBottom = 20;
		container.setLayout(gl);
		Global.initialize();

		GridData gd = new GridData(
				GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING | GridData.VERTICAL_ALIGN_END);
		gd.heightHint = 30;
		gd.widthHint = 180;
		Label label_title = new Label(container, SWT.NULL);
		label_title.setText("서비스 현황 트리");
		label_title.setLayoutData(gd);

		Composite compoiste_buttons = new Composite(container, SWT.NULL);
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_FILL | GridData.VERTICAL_ALIGN_CENTER);
		gd.heightHint = 40;
		gd.horizontalSpan = 3;
		compoiste_buttons.setLayoutData(gd);

		gl = new GridLayout();
		gl.makeColumnsEqualWidth = true;
		gl.numColumns = 9;
		compoiste_buttons.setLayout(gl);

		Button button_create = new Button(compoiste_buttons, SWT.PUSH);
		button_create.setImage(Global.image_Add);
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_END);
		gd.horizontalSpan = 7;
		button_create.setLayoutData(gd);
		button_create.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				Global.Service_Selected = new Model_Service();
				Dialog_Service_Edit dse = new Dialog_Service_Edit(getViewSite().getShell());
				Global.dialog_opened = dse;
				dse.open();
				treeviewer_Service.refresh();
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		Button button_edit = new Button(compoiste_buttons, SWT.PUSH);
		button_edit.setImage(Global.image_Service);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_edit.setLayoutData(gd);
		button_edit.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				if (treeviewer_Service.getStructuredSelection().isEmpty())
					return;

				IStructuredSelection is = treeviewer_Service.getStructuredSelection();
				File sel = (File) is.getFirstElement();
				if (sel.isDirectory())
					return;

				Global.Service_Selected = Global.Service_Temp;
				Dialog_Service_Edit dse = new Dialog_Service_Edit(getViewSite().getShell());
				Global.dialog_opened = dse;
				dse.open();
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		Button button_delete = new Button(compoiste_buttons, SWT.PUSH);
		button_delete.setImage(Global.image_delete);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		button_delete.setLayoutData(gd);
		button_delete.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {
				IStructuredSelection io = treeviewer_Service.getStructuredSelection();

				File f = (File) io.getFirstElement();

				File parent = f.getParentFile();
				if (f.exists()) {
					f.delete();
				}
				treeviewer_Service.setInput(ServiceRootfile.listFiles());
				treeviewer_Service.setSelection(new StructuredSelection(parent));
				treeviewer_Service.expandAll();
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 4;
		gd.heightHint = HEIGHT + 80;
		gd.widthHint = 300;

		ServiceRootfile = new File(Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/services");

		PatternFilter filter = new PatternFilter();

		FilteredTree tree = new FilteredTree(container, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL, filter,
				true);
		treeviewer_Service = tree.getViewer();
//		treeviewer_Service = new TreeViewer(container, SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.BORDER);
		treeviewer_Service.setContentProvider(new ServiceContentProvider());

		TreeViewerColumn treeColumn_service = new TreeViewerColumn(treeviewer_Service, SWT.NONE);

		treeColumn_service.getColumn().setText("Name");
		treeColumn_service.getColumn().setWidth(300);
		treeColumn_service.setLabelProvider(new DelegatingStyledCellLabelProvider(new FileNameProvider()));

		treeviewer_Service.setInput(ServiceRootfile.listFiles());
		tree.setLayoutData(gd);

		treeviewer_Service.addDoubleClickListener(new IDoubleClickListener() {

			@Override
			public void doubleClick(DoubleClickEvent event) {
				if (treeviewer_Service.getStructuredSelection().isEmpty())
					return;

				IStructuredSelection is = treeviewer_Service.getStructuredSelection();
				File sel = (File) is.getFirstElement();
				if (sel.isDirectory()) {

					treeviewer_Service.setExpandedState(sel, !treeviewer_Service.getExpandedState(sel));
					return;
				}

				Global.Service_Selected = Global.Service_Temp;
				Dialog_Service_Edit dse = new Dialog_Service_Edit(getViewSite().getShell());
				Global.dialog_opened = dse;
				dse.open();

				treeviewer_Service.refresh();

			}
		});

		treeviewer_Service.getTree().addKeyListener(new KeyAdapter() {

			@Override
			public void keyPressed(KeyEvent e) {
				String string = "";
				if ((e.stateMask & SWT.CTRL) != 0) {

					if (e.keyCode == 99) {

						IStructuredSelection is = treeviewer_Service.getStructuredSelection();
						Global.copied_file = (File) is.getFirstElement();

					} else if (e.keyCode == 118) {
						if (Global.copied_file != null) {
							String filename = Global.copied_file.getName();
							IStructuredSelection is = treeviewer_Service.getStructuredSelection();
							if (is.isEmpty() == false) {
								File f = (File) is.getFirstElement();
								if (f.isDirectory()) {

									String b = f.getAbsolutePath();

									File ff = new File(b + "/" + Global.copied_file.getName());
									if (ff.exists()) {
										if (filename.indexOf(".") != -1) {
											String tokens[] = filename.split("\\.");
											filename = tokens[0] + "(1)." + tokens[1];
										} else {
											filename = filename + "(1)";
										}
									}
									fileCopy(Global.copied_file.getAbsolutePath(), b + "/" + filename, true);
								} else if (f.getParentFile().isDirectory()) {
									String b = f.getParentFile().getAbsolutePath();
									File ff = new File(b + "/" + Global.copied_file.getName());
									if (ff.exists()) {
										if (filename.indexOf(".") != -1) {
											String tokens[] = filename.split("\\.");
											filename = tokens[0] + "(1)." + tokens[1];
										} else {
											filename = filename + "(1)";
										}
									}

									fileCopy(Global.copied_file.getAbsolutePath(), b + "/" + filename, true);
								}

							}

						}

					}
				} else if (e.keyCode == 127) {
					IStructuredSelection is = treeviewer_Service.getStructuredSelection();
					if (is.isEmpty() == false) {
						File f = (File) is.getFirstElement();

						f.delete();
					}
				}

				else {
					string += "CTRL - keyCode = " + e.keyCode;
					System.out.println(string);

				}
				treeviewer_Service.refresh();
			}

		});

		int operations = DND.DROP_MOVE;
		Transfer[] transferTypes = new Transfer[] { TextTransfer.getInstance() };
		treeviewer_Service.addDragSupport(operations, transferTypes, new MyDragListener(treeviewer_Service));
		treeviewer_Service.addDropSupport(operations, transferTypes, new MyDropListener(treeviewer_Service));

		treeviewer_Service.expandAll();
		gd = new GridData(GridData.HORIZONTAL_ALIGN_BEGINNING);
		gd.heightHint = 20;
		gd.horizontalSpan = 4;
		Label title2 = new Label(container, SWT.NULL);
		title2.setText("서비스 설명");
		title2.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.heightHint = 50;
		gd.horizontalSpan = 4;

		text_description = new Text(container, SWT.MULTI | SWT.WRAP | SWT.BORDER);
		text_description.setLayoutData(gd);

		gd = new GridData(GridData.HORIZONTAL_ALIGN_BEGINNING);
		gd.heightHint = 20;
		gd.horizontalSpan = 4;
		Label title3 = new Label(container, SWT.NULL);
		title3.setText("호출 API");
		title3.setLayoutData(gd);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 4;
		gd.heightHint = HEIGHT;
		gd.widthHint = WIDTH;

		tableviewer_selected_api = new TableViewer(container,
				SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		tableviewer_selected_api.setContentProvider(ArrayContentProvider.getInstance());

		createColumns(tableviewer_selected_api);

		final Table table = tableviewer_selected_api.getTable();
		table.setLayoutData(gd);
		table.setHeaderVisible(true);
		table.setLinesVisible(true);
		table.setEnabled(true);

		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		gd.heightHint = 30;
		gd.widthHint = WIDTH;

		treeviewer_Service.addSelectionChangedListener(new ISelectionChangedListener() {
			@Override
			public void selectionChanged(SelectionChangedEvent event) {

				IStructuredSelection is = event.getStructuredSelection();

				if (is.isEmpty())
					return;

				File file = (File) is.getFirstElement();

				boolean is_problem = false;
				if (file.isFile() == true && file.isDirectory() == false) {

					try {

						factory = DocumentBuilderFactory.newInstance();
						factory.setNamespaceAware(true);

						// 서비스 모
						Model_Service model_service = new Model_Service();
						Global.Service_Temp = model_service;
						model_service.file = file;
						model_service.file_path = file.getAbsolutePath();

						List<Model_Base> list_selected_apis = model_service.api_list;

						builder = factory.newDocumentBuilder();
						doc = builder.parse(file);

						Element element_root = doc.getDocumentElement();

						// description

						if (element_root.hasAttribute("name")) {
							String name = element_root.getAttribute("name");
							model_service.name = name;
						} else {
							String fullname = file.getName();
							model_service.name = fullname.substring(0, fullname.length() - 4);
						}

						if (element_root.hasAttribute("description")) {
							String description = element_root.getAttribute("description");
							text_description.setText(description);
							model_service.description = description;
						} else {
							text_description.setText("");
						}

						// api
						NodeList list_api = element_root.getElementsByTagName("service");

						ArrayList<Model_Base> notfound = new ArrayList<Model_Base>();
						ArrayList<Model_Base> duplicate = new ArrayList<Model_Base>();
						ArrayList<Model_Base> notfound_param = new ArrayList<Model_Base>();
						String error_message = "";

						for (int i = 0; i < list_api.getLength(); i++) {
							Node node = list_api.item(i);
							if (node.getNodeType() == Node.ELEMENT_NODE) {

								Element element = (Element) list_api.item(i);
								String apiName = element.getAttribute("name");
								Global.logger(apiName);
								String api_description = "";
								if (element.hasAttribute("description"))
									api_description = element.getAttribute("description");
								Model_Base model_api = new Model_Base(i, apiName, null);
//								model_api.description = "<" + apiName + ">\n\t" + api_description;
								model_api.description = api_description;
								String api_priority_str = "";

								if (element.hasAttribute("priority")) {
									api_priority_str = element.getAttribute("priority");
									try {
										model_api.priority = Double.parseDouble(api_priority_str);
									} catch (Exception e) {
										e.printStackTrace();
									}
								}

								// 파라미터
								/////////////////////////////////////

								NodeList list_param = element.getElementsByTagName("param");

								for (int j = 0; j < list_param.getLength(); j++) {
									Node n = list_param.item(j);
									if (n.getNodeType() == Node.ELEMENT_NODE) {
										Element e = (Element) list_param.item(j);
										Model_Base model_param = null;
										String io = e.getAttribute("io");
										String type = e.getAttribute("type");
										String name = e.getAttribute("name");
										Global.logger("           " + io + " @ " + type + name);

										boolean isIn = false;

										model_param = new Model_Base(isIn, type, name, model_api);

										if (io.equals("in")) {

											model_param.isIn = true;

											if (e.hasAttribute("ptype")) {
												String ptype = e.getAttribute("ptype");

												for (ParamType p : ParamType.values()) {
													if (ptype.equals(p.name())) {

														if (p.equals(ParamType.Combo)) {
														}
														model_param.ptype = p;
													}
												}
											} else {

												if (type.indexOf("[]") != -1) {
													model_param.ptype = ParamType.Vector;

												} else {

													if (Global.hashmap_type.containsKey(type)) {
														model_param.ptype = ParamType.Text;
													} else {
														model_param.ptype = ParamType.Class;
													}
												}

											}
										}

										String tag = "";
										if (e.hasAttribute("tag"))
											tag = e.getAttribute("tag");

										String default_str = e.getAttribute("default");

										String back_value = "";

										if (e.hasAttribute("back_value")) {
											back_value = e.getAttribute("back_value");
											model_param.back_value = back_value;
										}
										String condition = "";
										if (e.hasAttribute("condition")) {
											condition = e.getAttribute("condition");
											model_param.parse_condition(condition);
										}
										String description_str = "";
										if (e.hasAttribute("description")) {
											description_str = e.getAttribute("description");
										}
										model_param.tag = tag;
										model_param.value = default_str;
										model_param.description = description_str;

										model_api.child.add(model_param);
									}
								}

								///////////////////////////////////////

								if (Global.hashmap_api.containsKey(apiName)) {
									Model_Base model_api_origin = Global.hashmap_api.get(apiName);

									for (Model_Base m : model_api.child) {
										String token = m.type + m.name;
										if (model_api_origin.name_cache.contains(token) == false) {
											notfound_param.add(m);
											model_api.paramNotOK = true;
										}
									}
									model_api.parent = model_api_origin.parent;
									model_api.order = i + 1;
									model_api.isDefaultParam = true;
									list_selected_apis.add(model_api);
//									}

								} else {

									model_api.notFound = true;
									list_selected_apis.add(model_api);
									notfound.add(model_api);

								}

							}

						}

						if (duplicate.size() > 0) {
							error_message += "중복된 이름의 API : ";
							for (Model_Base m : duplicate) {
								error_message += m.toString() + " from " + m.parent.name;
							}
							error_message += "\n";
						}

						if (notfound.size() > 0) {
							error_message += "찾을 수 없는 API : ";
							for (Model_Base m : notfound) {
								error_message += m.toString() + "  ";
							}
							error_message += "\n";
						}

						if (notfound_param.size() > 0) {
							error_message += "찾을 수 없는 파라미터 : \n";
							for (Model_Base m : notfound_param) {
								error_message += "\t" + m.type + "   " + m.name + "   ( " + m.parent.name + ")" + "\n";
							}
							error_message += "\n";
						}

						if (duplicate.size() > 0 || notfound.size() > 0 || notfound_param.size() > 0) {
							is_problem = true;
						}

						if (is_problem == false) {
						}

						Model_Base[] arr = new Model_Base[list_selected_apis.size()];
						list_selected_apis.toArray(arr);
						tableviewer_selected_api.setInput(arr);

					} catch (Exception e) {
						e.printStackTrace();
						text_description.setText("");

						Global.Service_Temp = new Model_Service();
						String fullname = file.getName();
						Global.Service_Temp.name = fullname.substring(0, fullname.length() - 4);
						tableviewer_selected_api.setInput(new Model_Base[] {});
						return;
					}
				}
			}
		});

		Global.treeviewerService = treeviewer_Service;
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
				return Global.PADDING + p.getOrder();
			}

			@Override
			public Color getForeground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.notFound || p.paramNotOK) {
					return new Color(getSite().getShell().getDisplay(), new RGBA(128, 128, 128, 255));
				}

				return null;
			}

			@Override
			public Color getBackground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.notFound || p.paramNotOK) {
					return new Color(getSite().getShell().getDisplay(), new RGBA(128, 0, 0, 255));
				}

				return null;
			}
		});

		TableViewerColumn colName = new TableViewerColumn(viewer, SWT.BORDER);
		colName.getColumn().setWidth(150);
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
				if (p.notFound || p.paramNotOK) {
					return new Color(getSite().getShell().getDisplay(), new RGBA(128, 128, 128, 255));
				}

				return null;
			}

			@Override
			public Color getBackground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.notFound || p.paramNotOK) {
					return new Color(getSite().getShell().getDisplay(), new RGBA(128, 0, 0, 255));
				}

				return null;
			}
		});

	}

	private class ServiceContentProvider implements ITreeContentProvider {
		public void inputChanged(Viewer v, Object oldInput, Object newInput) {
		}

		@Override
		public void dispose() {
		}

		@Override
		public Object[] getElements(Object inputElement) {
			return (File[]) inputElement;
		}

		@Override
		public Object[] getChildren(Object parentElement) {
			File file = (File) parentElement;
			return file.listFiles();
		}

		@Override
		public Object getParent(Object element) {
			File file = (File) element;
			return file.getParentFile();
		}

		@Override
		public boolean hasChildren(Object element) {
			File file = (File) element;
			if (file.isDirectory()) {
				return true;
			}
			return false;
		}

	}

	class FileNameProvider extends LabelProvider implements IStyledLabelProvider {

		public FileNameProvider() {

		}

		@Override
		public StyledString getStyledText(Object element) {
			if (element instanceof File) {
				File file = (File) element;
				String[] files = file.list();

				String name = getFileName(file);

				if (files != null) {
					StyledString styledString = new StyledString(name);
					styledString.append(" ( " + files.length + " ) ", StyledString.COUNTER_STYLER);
					return styledString;
				}
				StyledString styledString = new StyledString(name.substring(0, name.length() - 4));
				return styledString;
			}
			return null;
		}

		@Override
		public Image getImage(Object element) {
			if (element instanceof File) {
				if (((File) element).isDirectory()) {
					return Global.image_Folder;
				} else if (((File) element).isFile()) {
					return Global.image_Service;
				}
			}

			return super.getImage(element);
		}

		@Override
		public void dispose() {
			// garbage collect system resources
			if (resourceManager != null) {
				resourceManager.dispose();
				resourceManager = null;
			}
		}

		protected ResourceManager getResourceManager() {
			if (resourceManager == null) {
				resourceManager = new LocalResourceManager(JFaceResources.getResources());
			}
			return resourceManager;
		}

		private String getFileName(File file) {
			String name = file.getName();
			return name.isEmpty() ? file.getPath() : name;
		}
	}

	public class MyDragListener implements DragSourceListener {

		private final TreeViewer viewer;

		public MyDragListener(TreeViewer viewer) {
			this.viewer = viewer;
		}

		@Override
		public void dragFinished(DragSourceEvent event) {
			System.out.println("Finshed Drag");
		}

		@Override
		public void dragSetData(DragSourceEvent event) {

			IStructuredSelection selection = viewer.getStructuredSelection();
			File firstElement = (File) selection.getFirstElement();

			if (TextTransfer.getInstance().isSupportedType(event.dataType)) {
				String s = firstElement.getAbsolutePath();
				event.data = s;
			}

		}

		@Override
		public void dragStart(DragSourceEvent event) {
			System.out.println("Start Drag");
		}

	}

	public class MyDropListener extends ViewerDropAdapter {

		@SuppressWarnings("unused")
		private final TreeViewer viewer;
		private String targetpath = "";

		public MyDropListener(TreeViewer viewer) {
			super(viewer);
			this.viewer = viewer;
		}

		@Override
		public void drop(DropTargetEvent event) {
			int location = this.determineLocation(event);
			File target = (File) determineTarget(event);
			this.targetpath = target.getAbsolutePath();
			String translatedLocation = "";
			switch (location) {
			case 1:
				translatedLocation = "Dropped before the target ";
				break;
			case 2:
				translatedLocation = "Dropped after the target ";
				break;
			case 3:
				translatedLocation = "Dropped on the target ";
				break;
			case 4:
				translatedLocation = "Dropped into nothing ";
				break;
			}
			System.out.println(translatedLocation);
			System.out.println("The drop was done on the element: " + target);
			super.drop(event);
		}

		@Override
		public boolean performDrop(Object data) {

			File file = new File(data.toString());

			if ((new File(targetpath).isDirectory())) {
				fileCopy(data.toString(), targetpath + "/" + file.getName(), false);
			}

			treeviewer_Service.refresh();
			treeviewer_Service.setSelection(new StructuredSelection(new File(targetpath + "/" + file.getName())));

			return false;
		}

		@Override
		public boolean validateDrop(Object target, int operation, TransferData transferType) {
			return true;

		}

	}

	@Override
	public void setFocus() {
		Global.view_service = this;
		Global.initialize();
	}

	public static void fileCopy(String inFileName, String outFileName, boolean copy) {
		try {

			File inFile = new File(inFileName);
			File outFile = new File(outFileName);

			if (inFile.getAbsolutePath().equals(outFile.getAbsolutePath())) {
				return;
			}

			FileInputStream fis = new FileInputStream(inFileName);
			FileOutputStream fos = new FileOutputStream(outFileName);

			int data = 0;
			while ((data = fis.read()) != -1) {
				fos.write(data);
			}
			fis.close();
			fos.close();
			if (copy == false) {
				inFile.delete();
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}