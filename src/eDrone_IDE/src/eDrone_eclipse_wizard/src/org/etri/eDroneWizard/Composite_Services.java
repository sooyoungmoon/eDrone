package org.etri.eDroneWizard;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

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
import org.eclipse.jface.viewers.StyledString;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.TreeViewerColumn;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.RGBA;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.Tree;
import org.eclipse.swt.widgets.TreeItem;
import org.eclipse.ui.dialogs.FilteredTree;
import org.etri.eDrone.FilePatternFilter;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;
import org.etri.eDroneModel.ParamType;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class Composite_Services {

	public Composite parent;
	private TreeViewer treeviewer_Service;
	private TableViewer tableviewer_API;

	private DocumentBuilderFactory factory;
	private DocumentBuilder builder = null;
	private Document doc = null;

	private final int HEIGHT = 230;
	private final int WIDTH = 400;

	public Model_Service model_Service = null;

	public Composite_Services(Composite parent) {
		this.parent = parent;

		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		gd.heightHint = HEIGHT;
		gd.widthHint = WIDTH;

		File file = new File(Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/services");

		FilePatternFilter filter = new FilePatternFilter();

		FilteredTree tree = new FilteredTree(parent, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL, filter,
				true);
		treeviewer_Service = tree.getViewer();

		treeviewer_Service.setContentProvider(new ServiceContentProvider());
		Global.wizard_page1.treeviewer_service = treeviewer_Service;

		TreeViewerColumn treeColumn_service = new TreeViewerColumn(treeviewer_Service, SWT.NONE);

		treeColumn_service.getColumn().setText("Name");
		treeColumn_service.getColumn().setWidth(300);
		treeColumn_service.setLabelProvider(new DelegatingStyledCellLabelProvider(new FileNameProvider()));

		treeviewer_Service.setInput(file.listFiles());
		treeviewer_Service.expandAll();

		tree.setLayoutData(gd);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 2;
		gd.heightHint = HEIGHT;
		gd.widthHint = WIDTH;

		tableviewer_API = new TableViewer(parent,
				SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		tableviewer_API.setContentProvider(ArrayContentProvider.getInstance());

		createColumns(tableviewer_API);

		final Table table = tableviewer_API.getTable();
		table.setLayoutData(gd);
		table.setHeaderVisible(true);
		table.setLinesVisible(true);
		table.setEnabled(true);
		treeviewer_Service.addDoubleClickListener(new IDoubleClickListener() {

			@Override
			public void doubleClick(DoubleClickEvent event) {
				if (treeviewer_Service.getStructuredSelection().isEmpty())
					return;

				IStructuredSelection is = treeviewer_Service.getStructuredSelection();
				File sel = (File) is.getFirstElement();
				if (sel.isDirectory()) {

					treeviewer_Service.setExpandedState(sel, !treeviewer_Service.getExpandedState(sel));
				}

			}
		});
		treeviewer_Service.addSelectionChangedListener(new ISelectionChangedListener() {
			@Override
			public void selectionChanged(SelectionChangedEvent event) {

				TreeViewer viewer = (TreeViewer) event.getSource();

				Tree tree = viewer.getTree();
				TreeItem[] items = tree.getSelection();

				if (items.length < 1) {
					System.out.println("tree item length 0");
					return;
				}
				TreeItem item = items[0];
				File file = (File) item.getData();

				if (file.isDirectory() == true) {
					return;
				}

				boolean is_problem = false;
				if (file.isFile() == true && file.isDirectory() == false) {

					try {

						factory = DocumentBuilderFactory.newInstance();
						factory.setNamespaceAware(true);

						// 서비스 모
						Model_Service model_service = new Model_Service();
						Global.Service_Temp = model_service;
						model_service.file_path = file.getAbsolutePath();

						List<Model_Base> list_selected_apis = model_service.api_list;

						builder = factory.newDocumentBuilder();
						doc = builder.parse(file);

						Element element_root = doc.getDocumentElement();

						// description
						if (element_root.hasAttribute("description")) {
							String description = element_root.getAttribute("description");
							Global.wizard_page1.text_ServiceDescription.setText(description);
							Global.wizard_page1.composite_title_03.pack();
							model_service.description = description;
						} else {
							Global.wizard_page1.text_ServiceDescription.setText("");
							Global.wizard_page1.composite_title_03.pack();
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
								model_api.description = "<" + apiName + ">\n\t" + api_description;
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
										if (io.equals("in")) {
											isIn = true;
										}

										model_param = new Model_Base(isIn, type, name, model_api);
										if (io.equals("in")) {
											if (e.hasAttribute("ptype")) {
												String ptype = e.getAttribute("ptype");

												for (ParamType p : ParamType.values()) {
													if (ptype.equals(p.name())) {

														if (p.equals(ParamType.Combo)) {
															System.out.println("ha");
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

										String default_str = "";
										if (e.hasAttribute("default")) {
											String tmp = e.getAttribute("default");
											if (tmp.trim().length() > 0) {
												default_str = tmp;
											}
											default_str = tmp;
										}
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
							Global.wizard_page1.isAPIClear = false;
							Global.wizard_page1.dialogChanged();
							Global.wizard_page1.set_warning_message(error_message);
							Global.wizard_page1.updateStatus("오류 메시지를 확인하세요");
						}

						if (is_problem == false) {
							Global.wizard_page1.isAPIClear = true;
							Global.wizard_page1.dialogChanged();
							Global.wizard_page1.set_warning_message("");
						}

						Model_Base[] arr = new Model_Base[list_selected_apis.size()];
						list_selected_apis.toArray(arr);
						tableviewer_API.setInput(arr);

					} catch (Exception e) {
						e.printStackTrace();
						Global.wizard_page1.text_ServiceDescription.setText("");
						Global.wizard_page1.composite_title_03.pack();
						Global.wizard_page1.isAPIClear = false;
						Global.wizard_page1.updateStatus("올바르지 않은 형식의 파일입니다.");
						tableviewer_API.setInput(new Node[] {});
						System.out.println(e.getMessage());
					}
				}
			}
		});
	}

	private void createColumns(TableViewer viewer) {

		TableViewerColumn colOrder = new TableViewerColumn(viewer, SWT.BORDER);
		colOrder.getColumn().setWidth(55);
		colOrder.getColumn().setText("Order");
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
					return new Color(parent.getDisplay(), new RGBA(128, 128, 128, 255));
				}

				return null;
			}

			@Override
			public Color getBackground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.notFound || p.paramNotOK) {
					return new Color(parent.getDisplay(), new RGBA(128, 0, 0, 255));
				}

				return null;
			}
		});

		TableViewerColumn colName = new TableViewerColumn(viewer, SWT.BORDER);
		colName.getColumn().setWidth(300);
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
					return new Color(parent.getDisplay(), new RGBA(128, 128, 128, 255));
				}

				return null;
			}

			@Override
			public Color getBackground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.notFound || p.paramNotOK) {
					return new Color(parent.getDisplay(), new RGBA(128, 0, 0, 255));
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
			return null;
		}

		private String getFileName(File file) {
			String name = file.getName();
			return name.isEmpty() ? file.getPath() : name;
		}
	}

}
