package org.etri.eDroneView.Template;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.TitleAreaDialog;
import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.FocusEvent;
import org.eclipse.swt.events.FocusListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.Text;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Section;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

public class Dialog_Section extends TitleAreaDialog {

	private TableViewer viewer;

	private ArrayList<Model_Section> temp_list = new ArrayList<Model_Section>();

	private HashMap<String, String> hmap_changed = new HashMap<String, String>();
	public Text text_annotation;

	private Model_Section msectionLastSelected;

	public Dialog_Section(Shell parentShell) {
		super(parentShell);
	}

	@Override
	public void create() {
		super.create();
		setTitle("섹션 관리");
//		setMessage("파라미터 " + model.name + " 조건 조회", IMessageProvider.INFORMATION);
	}

	@Override
	protected void createButtonsForButtonBar(final Composite parent) {
		super.createButton(parent, IDialogConstants.OK_ID, IDialogConstants.OK_LABEL, true);
		super.createButton(parent, IDialogConstants.CANCEL_ID, IDialogConstants.CANCEL_LABEL, true);
		
		
		Button button = super.getButton(IDialogConstants.OK_ID);
		GridData gd =(GridData)button.getLayoutData();
		gd.widthHint =200;
		button.setText("Apply Changes");
	}

	@Override
	protected Control createDialogArea(Composite parent) {

		initialize();

		Composite area = (Composite) super.createDialogArea(parent);
		Composite container = new Composite(area, SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		GridLayout layout = new GridLayout(2, false);
		layout.marginLeft = 10;
		layout.marginRight = 10;
		container.setLayout(layout);

		Composite compoiste_buttons = new Composite(container, SWT.NULL);
		GridData gd = new GridData(
				GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_FILL | GridData.VERTICAL_ALIGN_CENTER);
		gd.heightHint = 40;
		gd.horizontalSpan = 2;
		compoiste_buttons.setLayoutData(gd);

		layout = new GridLayout();
		layout.makeColumnsEqualWidth = true;
		layout.numColumns = 10;
		compoiste_buttons.setLayout(layout);

		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.HORIZONTAL_ALIGN_BEGINNING | GridData.VERTICAL_ALIGN_END);
		gd.horizontalSpan = 8;
		gd.heightHint = 20;
		Label dummy = new Label(compoiste_buttons, SWT.NULL);
		dummy.setText("Section List");
		dummy.setLayoutData(gd);

		Button button_create = new Button(compoiste_buttons, SWT.PUSH);
		button_create.setImage(Global.image_Add);
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.VERTICAL_ALIGN_END);
		button_create.setLayoutData(gd);
		button_create.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				Model_Section new_section = new Model_Section("new section", "this is new section");
				new_section.order = temp_list.size() + 1;

				Dialog_Section_Name dsn = new Dialog_Section_Name(getShell(), new_section);
				if (dsn.open() == 0) {
					temp_list.add(new_section);
					viewer.setInput(temp_list.toArray());
					viewer.setSelection(new StructuredSelection(new_section));
					viewer.getTable().showSelection();
				}
			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		Button button_delete = new Button(compoiste_buttons, SWT.PUSH);
		button_delete.setImage(Global.image_delete);
		gd = new GridData(GridData.FILL_HORIZONTAL | GridData.VERTICAL_ALIGN_END);
		button_delete.setLayoutData(gd);
		button_delete.addSelectionListener(new SelectionListener() {

			@Override
			public void widgetSelected(SelectionEvent e) {

				temp_list.remove(viewer.getTable().getSelectionIndex());

				for (int i = 0; i < temp_list.size(); i++) {

					temp_list.get(i).order = i + 1;

				}

				viewer.setInput(temp_list.toArray());

			}

			@Override
			public void widgetDefaultSelected(SelectionEvent e) {

			}
		});

		viewer = new TableViewer(container, SWT.MULTI | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		viewer.setContentProvider(new ArrayContentProvider());

		createColumns(viewer);

		gd = new GridData(GridData.FILL_BOTH);
		gd.heightHint = 150;
		gd.verticalSpan = 2;

		final Table table = viewer.getTable();
		table.setLayoutData(gd);
		table.setHeaderVisible(false);
		table.setLinesVisible(true);
		table.setEnabled(true);

		viewer.addSelectionChangedListener(new ISelectionChangedListener() {

			@Override
			public void selectionChanged(SelectionChangedEvent event) {
				IStructuredSelection selection = (IStructuredSelection) event.getSelection();

				Model_Section model = (Model_Section) selection.getFirstElement();
				if (model == null)
					return;

				if (msectionLastSelected == null) {
					msectionLastSelected = model;
				} else {
					msectionLastSelected.annotation = text_annotation.getText().substring(1);
					msectionLastSelected = model;
				}
				text_annotation.setText("\t" + model.annotation);

			}
		});

		viewer.addDoubleClickListener(new IDoubleClickListener() {

			@Override
			public void doubleClick(DoubleClickEvent event) {

				IStructuredSelection selection = (IStructuredSelection) event.getSelection();

				Model_Section model = (Model_Section) selection.getFirstElement();
				Dialog_Section_Name dsn = new Dialog_Section_Name(getShell(), model);
				if (dsn.open() == 0) {
					viewer.refresh();
				}
			}
		});

		viewer.setInput(temp_list.toArray());
		createArrowButton(container, SWT.UP, sel_listner_button_move_up);
		createArrowButton(container, SWT.DOWN, sel_listner_button_move_down);

		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		Label title = new Label(container, SWT.NULL);
		title.setText("Annotation");
		gd.heightHint = 25;
		title.setLayoutData(gd);

		gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 2;
		text_annotation = new Text(container, SWT.MULTI | SWT.WRAP | SWT.BORDER);
		gd.heightHint = 100;
		text_annotation.setLayoutData(gd);
//		text_annotation.addFocusListener(new FocusListener() {
//
//			@Override
//			public void focusLost(FocusEvent e) {
//
//				if (msectionLastSelected != null) {
//					msectionLastSelected.annotation = text_annotation.getText().substring(1);
//				}
//
//				IStructuredSelection selection = (IStructuredSelection) viewer.getSelection();
//				Model_Section model = (Model_Section) selection.getFirstElement();
//				msectionLastSelected = model;
//
//			}
//
//			@Override
//			public void focusGained(FocusEvent e) {
//
//			}
//		});

		return area;

	}

	private void initialize() {

		for (Model_Section s : Global.list_sections) {

			Model_Section ms = new Model_Section(s);
			temp_list.add(ms);

			hmap_changed.put(s.name, "");
		}
	}

	public void pack() {

		TableColumn[] cols = viewer.getTable().getColumns();
		for (int i = 0; i < cols.length; i++) {
			cols[i].pack();
		}
	}

	public void createArrowButton(Composite parent, int alignment, SelectionListener selectionListener) {

		final GridData gd = new GridData();
		gd.heightHint = 100;
		final Button arwBtn = new Button(parent, SWT.ARROW);
		arwBtn.setAlignment(alignment);
		arwBtn.addSelectionListener(selectionListener);
		arwBtn.setLayoutData(gd);

	}

	SelectionListener sel_listner_button_move_up = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {

			int index = viewer.getTable().getSelectionIndex();
			if (index == -1 || index == 0)
				return;

			temp_list.get(index).order -= 1;
			temp_list.get(index - 1).order += 1;
			Collections.swap(temp_list, index - 1, index);
			viewer.setInput(temp_list.toArray());
			viewer.setSelection(new StructuredSelection(temp_list.get(index - 1)));
			viewer.getTable().showSelection();
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}
	};

	SelectionListener sel_listner_button_move_down = new SelectionListener() {

		@Override
		public void widgetSelected(SelectionEvent e) {
			int index = viewer.getTable().getSelectionIndex();

			System.out.println(viewer.getTable().getItemCount());
			if (index == -1 || index >= viewer.getTable().getItemCount() - 1)
				return;

			temp_list.get(index).order += 1;
			temp_list.get(index + 1).order -= 1;
			Collections.swap(temp_list, index + 1, index);
			viewer.setInput(temp_list.toArray());
			viewer.setSelection(new StructuredSelection(temp_list.get(index + 1)));
			viewer.getTable().showSelection();
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {

		}
	};

	private void createColumns(TableViewer viewer) {

		TableViewerColumn colHasContents = new TableViewerColumn(viewer, SWT.BORDER);
		colHasContents.getColumn().setWidth(40);
		colHasContents.getColumn().setText("   ");
		colHasContents.getColumn().setAlignment(SWT.CENTER);
		colHasContents.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Section p = (Model_Section) element;
				return Integer.toString(p.order);
			}

		});

		TableViewerColumn colName = new TableViewerColumn(viewer, SWT.BORDER);
		colName.getColumn().setWidth(180);
		colName.getColumn().setText("Section Name");
		colName.getColumn().setAlignment(SWT.LEFT);
		colName.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {

				Model_Section p = (Model_Section) element;
				return p.name;
			}
		});

	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void okPressed() {

		boolean isDirty = false;
		for (Model_Section ms : temp_list) {

			if (ms.origin_name.equals(ms.name) == false) {
				hmap_changed.put(ms.origin_name, ms.name);
				isDirty = true;
			}

		}

		if (isDirty == false) {
			super.okPressed();
			return;
		}

		if (MessageDialog.openQuestion(getShell(), "섹션 관리", "변경사항을 저장하시겠습니까?")) {

			write_section_info();

			File dir = new File(Global.CATKIN_WORKSPACE_PATH + "/" + Global.DEPNEDING_FILES_PATH + "/template/source");

			if (dir.exists()) {

				FilenameFilter filter = new FilenameFilter() {

					@Override
					public boolean accept(File dir, String name) {
						if (name.endsWith(".tpl")) {
							return true;
						}
						return false;
					}
				};

				FileReader reader = null;
				BufferedReader buf_reader = null;
				FileWriter writer = null;
				BufferedWriter buf_writer = null;

				String line = "";

				File cacheFile = new File(dir.getAbsolutePath() + "/.cache.txt");

				for (File file : dir.listFiles(filter)) {

					try {
						reader = new FileReader(file);
						buf_reader = new BufferedReader(reader);
						writer = new FileWriter(cacheFile);
						buf_writer = new BufferedWriter(writer);

						while ((line = buf_reader.readLine()) != null) {

							if (line.indexOf("---") != -1) {

								String temp = line.trim();

								if (temp.startsWith("---")) {

									temp = temp.substring(3, temp.length());
									if (temp.indexOf(",") != -1) {

										String tokens[] = temp.split(",");
										temp = tokens[0];

									}

									if (hmap_changed.containsKey(temp) && hmap_changed.get(temp).length() > 0) {

										line = line.replace(temp, hmap_changed.get(temp));

									}

								}

							}
							buf_writer.write(line);
							buf_writer.newLine();
						}

					} catch (IOException e) {
						e.printStackTrace();
					} finally {

						// BufferedReader FileReader를 닫아준다.
						if (buf_reader != null) {
							try {
								buf_reader.close();
							} catch (IOException e) {
							}
						}
						if (reader != null) {
							try {
								reader.close();
							} catch (IOException e) {
							}
						}
						// BufferedWriter FileWriter를 닫아준다.
						if (buf_writer != null) {
							try {
								buf_writer.close();
							} catch (IOException e) {
							}
						}
						if (writer != null) {
							try {
								writer.close();
							} catch (IOException e) {
							}
						}
					}

					cacheFile.renameTo(file);

				}

				if (cacheFile.exists())
					cacheFile.delete();
			}

		}

		super.okPressed();
	}

	private void write_section_info() {

		String path = Global.CATKIN_WORKSPACE_PATH + Global.DEPNEDING_FILES_PATH + "/template";

		File xmlFile = new File(path + "/source/section.xml");
		File tplFile = new File(path + "/source/section.tpl");

		DocumentBuilderFactory factory;
		DocumentBuilder builder;
		Document doc;

		try {
			factory = DocumentBuilderFactory.newInstance();
			factory.setNamespaceAware(true);
			builder = factory.newDocumentBuilder();
			doc = builder.newDocument();

			Element rootElement = doc.createElement("root");
			doc.appendChild(rootElement);

			for (Model_Section s : temp_list) {
				Element el = doc.createElement("section");
				el.setAttribute("name", s.name);
				el.setAttribute("annotation", s.annotation);
				if (s.name.equals("main") || s.name.equals("end")) {
					el.setTextContent(s.content_stringbuilder.toString());
				}
				rootElement.appendChild(el);
			}

			TransformerFactory transformerFactory = TransformerFactory.newInstance();
			Transformer transformer = transformerFactory.newTransformer();

			transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
			transformer.setOutputProperty(OutputKeys.INDENT, "yes");
			transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
			DOMSource source = new DOMSource(doc);
			StreamResult result = new StreamResult(new FileOutputStream(xmlFile));

			transformer.transform(source, result);

		} catch (Exception e) {
			MessageDialog.openWarning(getShell(), "Warning", "템플릿 저장에 실패했습니다");
			e.printStackTrace();
			return;
		}

		try {
			FileWriter writer = new FileWriter(tplFile);
			StringBuilder sb = new StringBuilder();
			for (Model_Section s : temp_list) {

				sb.append("\n\n/*" + s.annotation + "*/");

				if (s.name.equals("section_main")) {
					sb.append("\nint main (int argc, char** argv)\r\n" + "{\r\n\n" + "");
				} else if (s.name.equals("section_end")) {
					sb.append("\nreturn 0;\r\n" + "\r\n" + "}");
				} else {
					sb.append("\n--" + s.name.trim() + "\n\n");
				}
			}

			writer.write(sb.toString());
			writer.close();
		} catch (IOException e) {
			MessageDialog.openWarning(getShell(), "Warning", "템플릿 저장에 실패했습니다");
			e.printStackTrace();
			return;
		}

	}
}