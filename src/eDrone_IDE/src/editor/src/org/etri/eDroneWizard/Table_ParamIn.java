package org.etri.eDroneWizard;

import org.eclipse.jface.bindings.keys.IKeyLookup;
import org.eclipse.jface.bindings.keys.KeyLookupFactory;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ArrayContentProvider;
import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.CheckboxCellEditor;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.ColumnViewerEditor;
import org.eclipse.jface.viewers.ColumnViewerEditorActivationEvent;
import org.eclipse.jface.viewers.ColumnViewerEditorActivationStrategy;
import org.eclipse.jface.viewers.ColumnViewerToolTipSupport;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.EditingSupport;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.TableViewerEditor;
import org.eclipse.jface.window.ToolTip;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.RGBA;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.TableItem;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.ParamType;
import org.etri.eDroneView.Service.Dialog_ParamIn_Class;
import org.etri.eDroneView.Service.Dialog_ParamIn_Vector;

/**
 * 
 * 두번째 페이지 서비스 탭에 파라미터 테이블 뷰어를 구현한다.
 * 
 */

public class Table_ParamIn {

	private static TableViewer table_viewer;
	private Composite parent = null;

	public void setFocus() {
		table_viewer.getControl().setFocus();
	}

	public Table_ParamIn(Composite parent) {

		table_viewer = new TableViewer(parent,
				SWT.SINGLE | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);
		this.parent = parent;
		createColumns(parent, table_viewer);

		Table table = table_viewer.getTable();
		table.setLinesVisible(true);
		table.setHeaderVisible(true);

		table.addListener(SWT.PaintItem, new Listener() {

			@Override
			public void handleEvent(Event event) {
				if (event.index == 0) {
					Image tmpImage = null;
					int tmpWidth = 0;
					int tmpHeight = 0;
					int tmpX = 0;
					int tmpY = 0;

					tmpWidth = table.getColumn(event.index).getWidth();
					tmpHeight = ((TableItem) event.item).getBounds().height;
					TableItem item = (TableItem) event.item;
					Model_Base model = (Model_Base) item.getData();
					if (model.isArg) {
						tmpImage = Global.image_Checked;
					} else {
						tmpImage = Global.image_UnChecked;
					}

					tmpX = tmpImage.getBounds().width;
					tmpX = (tmpWidth / 2 - tmpX / 2);
					tmpY = tmpImage.getBounds().height;
					tmpY = (tmpHeight / 2 - tmpY / 2);
					if (tmpX <= 0)
						tmpX = event.x;
					else
						tmpX += event.x;
					if (tmpY <= 0)
						tmpY = event.y;
					else
						tmpY += event.y;
					event.gc.drawImage(tmpImage, tmpX, tmpY);

				}

				if (event.index == 4) {
					Image tmpImage = null;
					int tmpWidth = 0;
					int tmpHeight = 0;
					int tmpX = 0;
					int tmpY = 0;

					tmpWidth = table.getColumn(event.index).getWidth();
					tmpHeight = ((TableItem) event.item).getBounds().height;
					TableItem item = (TableItem) event.item;
					Model_Base model = (Model_Base) item.getData();

					if (model.hasCondition == false && model.isConditionSatisfied)
						return;

					if (model.isConditionSatisfied || model.isArg) {
						tmpImage = Global.image_Accepted;
					} else {
						tmpImage = Global.image_Denied;
					}

					tmpX = tmpImage.getBounds().width;
					tmpX = (tmpWidth / 2 - tmpX / 2);
					tmpY = tmpImage.getBounds().height;
					tmpY = (tmpHeight / 2 - tmpY / 2);
					if (tmpX <= 0)
						tmpX = event.x;
					else
						tmpX += event.x;
					if (tmpY <= 0)
						tmpY = event.y;
					else
						tmpY += event.y;
					event.gc.drawImage(tmpImage, tmpX, tmpY);

				}
			}
		});

		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.horizontalSpan = 7;
		gd.heightHint = 120;
		gd.grabExcessHorizontalSpace = true;
		table_viewer.getControl().setLayoutData(gd);
		table_viewer.setContentProvider(new ArrayContentProvider());
		ColumnViewerToolTipSupport.enableFor(table_viewer, ToolTip.NO_RECREATE);
		ColumnViewerEditorActivationStrategy activationSupport = new ColumnViewerEditorActivationStrategy(
				table_viewer) {
			@Override
			protected boolean isEditorActivationEvent(ColumnViewerEditorActivationEvent event) {
				return event.eventType == ColumnViewerEditorActivationEvent.TRAVERSAL
						|| event.eventType == ColumnViewerEditorActivationEvent.MOUSE_CLICK_SELECTION
						|| event.eventType == ColumnViewerEditorActivationEvent.PROGRAMMATIC
						|| (event.eventType == ColumnViewerEditorActivationEvent.KEY_PRESSED
								&& event.keyCode == KeyLookupFactory.getDefault()
										.formalKeyLookup(IKeyLookup.ENTER_NAME));
			}
		};
		activationSupport.setEnableEditorActivationWithKeyboard(false);
		TableViewerEditor.create(table_viewer, activationSupport,
				ColumnViewerEditor.TABBING_VERTICAL | ColumnViewerEditor.KEYBOARD_ACTIVATION);

		table_viewer.addDoubleClickListener(new IDoubleClickListener() {
			@Override
			public void doubleClick(DoubleClickEvent event) {
				IStructuredSelection is = (IStructuredSelection) event.getSelection();
				Model_Base model = (Model_Base) is.getFirstElement();

				if (model == null)
					return;
				System.out.println(model.name);

				if (model.ptype.equals(ParamType.Text)) {
					Dialog_ParamIn_Text dpt = new Dialog_ParamIn_Text(parent.getShell(), model);
					dpt.open();
				} else if (model.ptype.equals(ParamType.Combo)) {
					Dialog_ParamIn_Combo dpcb = new Dialog_ParamIn_Combo(parent.getShell(), model);
					dpcb.open();
				} else if (model.ptype.equals(ParamType.Class)) {
					Dialog_ParamIn_Class dpc = new Dialog_ParamIn_Class(parent.getShell(), model, false);
					dpc.open();
				} else if (model.ptype.equals(ParamType.Vector)) {
					Dialog_ParamIn_Vector dpv = new Dialog_ParamIn_Vector(parent.getShell(), model, false);
					dpv.open();
				} else {

				}

				Global.validate_service(true);

				table_viewer.refresh();

			}

		});
		Global.wizard_page2.tableviewer_paramin = table_viewer;
	}

	private TableViewerColumn createTableViewerColumn(String title, int bound) {

		int align = SWT.CENTER;
		if (title.equals("Description")) {
			align = SWT.BEGINNING;
		}
		final TableViewerColumn viewerColumn = new TableViewerColumn(table_viewer, align);
		final TableColumn column = viewerColumn.getColumn();
		column.setText(title);
		column.setWidth(bound);
		column.setResizable(true);
		column.setMoveable(true);
		return viewerColumn;
	}

	private void createColumns(final Composite parent, final TableViewer viewer) {

		String[] titles = { "Arg", "Type", "Name", "Value", "Conditions", "Description" };
		int[] bounds = { 45, 100, 150, 100, 100, 100, 600 };

		TableViewerColumn col = createTableViewerColumn(titles[0], bounds[0]);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return null;
			}

			@Override
			public String getToolTipText(Object element) {

				return "Tooltip ( " + "체크시 해당 변수는 매개변수로 설정됩니다" + " )";
			}
		});
		col.setEditingSupport(new ArgCheckEditingSupport(viewer));

		col = createTableViewerColumn(titles[1], bounds[1]);
		col.setLabelProvider(get_ColLabelProvider(1, false, null));

		col = createTableViewerColumn(titles[2], bounds[2]);
		col.setLabelProvider(get_ColLabelProvider(2, false, null));

		col = createTableViewerColumn(titles[3], bounds[3]);
		col.setLabelProvider(get_ColLabelProvider(3, false, null));

		col = createTableViewerColumn(titles[4], bounds[4]);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return null;
			}

			@Override
			public Color getBackground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.isArg) {
					return new Color(parent.getDisplay(), new RGBA(128, 128, 128, 255));
				}
				return super.getBackground(element);
			}

			@Override
			public Color getForeground(Object element) {
				Model_Base p = (Model_Base) element;
				if (p.isArg) {
					return new Color(parent.getDisplay(), new RGBA(255, 255, 255, 255));
				}
				return super.getForeground(element);
			}
		});
		col.setEditingSupport(new ConditionEditingSupport(viewer));

		col = createTableViewerColumn(titles[5], bounds[5]);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base p = (Model_Base) element;
				return p.description;
			}

			@Override
			public String getToolTipText(Object element) {
				Model_Base p = (Model_Base) element;
				return p.description;
			}

			@Override
			public Color getBackground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.isArg) {
					return new Color(parent.getDisplay(), new RGBA(128, 128, 128, 255));
				}
				return super.getBackground(element);
			}

			@Override
			public Color getForeground(Object element) {
				Model_Base p = (Model_Base) element;
				if (p.isArg) {
					return new Color(parent.getDisplay(), new RGBA(255, 255, 255, 255));
				}
				return super.getForeground(element);
			}
		});
	}

	private ColumnLabelProvider get_ColLabelProvider(int num, boolean hasTooltip, String tooltip_message) {

		ColumnLabelProvider ColLabelProvider = new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base p = (Model_Base) element;
				String text = "";
				switch (num) {
				case 1:
					text = p.type;
					break;
				case 2:
					text = p.name;
					break;
				case 3:
					text = p.value;
					if (p.isArg) {
						text = "매개변수";
					} else if (p.ptype.equals(ParamType.Text)) {
						if (text.indexOf(";") != -1) {
							String tokens[] = text.split(";");
							text = tokens[tokens.length - 1];
							if (text.equals("None"))
								return null;
						}
					} else if (p.ptype.equals(ParamType.Vector)) {
						String tokens[] = text.split(";");
						text = tokens[tokens.length - 1];
						if (text.equals("None")) {
							return null;
						}
					} else if (p.ptype.equals(ParamType.Class)) {
						if (text.replace(" ", "").length() < 1)
							break;
						String tokens[] = text.split(";");

						StringBuilder sb = new StringBuilder();
						if (tokens.length < 1) {
							text = "";
							break;
						}
						int i = 1;
						for (; i < tokens.length - 1; i++) {
							String ts[] = tokens[i].split(",");
							String target = ts[2];
							if (target.equals("None")) {
								target = " ";
							}
							sb.append(target);
							sb.append(",");
						}
						String ts[] = tokens[i].split(",");
						String target = ts[2];
						if (target.equals("None")) {
							target = " ";
						}
						sb.append(target);
						text = sb.toString();
					}
					break;
				case 4:
					break;
				case 5:
					text = "   " + p.description;
					break;
				}
				return text;
			}

			@Override
			public String getToolTipText(Object element) {

				if (hasTooltip == true)
					return "Tooltip (" + tooltip_message + ")";
				else {
					return null;
				}
			}

			@Override
			public Color getBackground(Object element) {

				Model_Base p = (Model_Base) element;
				if (p.isArg) {
					return new Color(parent.getDisplay(), new RGBA(128, 128, 128, 255));
				}
				return super.getBackground(element);
			}

			@Override
			public Color getForeground(Object element) {
				Model_Base p = (Model_Base) element;
				if (p.isArg) {
					return new Color(parent.getDisplay(), new RGBA(255, 255, 255, 255));
				}
				return super.getForeground(element);
			}
		};
		return ColLabelProvider;

	}

	public static class ArgCheckEditingSupport extends EditingSupport {

		private final TableViewer viewer;

		public ArgCheckEditingSupport(TableViewer viewer) {
			super(viewer);
			this.viewer = viewer;
		}

		@Override
		protected CellEditor getCellEditor(Object element) {
			return new CheckboxCellEditor(null, SWT.CHECK | SWT.READ_ONLY);
		}

		@Override
		protected boolean canEdit(Object element) {
			return true;
		}

		@Override
		protected Object getValue(Object element) {
			Model_Base p = (Model_Base) element;
			return p.isArg;

		}

		@Override
		protected void setValue(Object element, Object value) {
			Model_Base p = (Model_Base) element;
			p.isArg = !p.isArg;
			viewer.update(element, null);
			Global.validate_service(true);
		}
	}

	public static class ConditionEditingSupport extends EditingSupport {

		private final TableViewer viewer;

		public ConditionEditingSupport(TableViewer viewer) {
			super(viewer);
			this.viewer = viewer;
		}

		@Override
		protected CellEditor getCellEditor(Object element) {
			return new CheckboxCellEditor(null, SWT.CHECK | SWT.READ_ONLY);
		}

		@Override
		protected boolean canEdit(Object element) {
			return true;
		}

		@Override
		protected Object getValue(Object element) {
			Model_Base model = (Model_Base) element;
			if (model.hasCondition) {
				Dialog_Condition dialog = new Dialog_Condition(viewer.getTable().getShell(), model);
				dialog.open();
			} else if (model.hasCondition == false && model.isConditionSatisfied == false) {
				MessageDialog.openError(viewer.getTable().getShell(), "Warning", "매개변수가 아닌 변수의 값은 비어있을 수 없습니다.");
			}
			return model.isConditionSatisfied;

		}

		@Override
		protected void setValue(Object element, Object value) {
			viewer.update(element, null);
		}
	}
}