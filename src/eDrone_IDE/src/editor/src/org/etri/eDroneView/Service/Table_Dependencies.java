package org.etri.eDroneView.Service;

import java.util.ArrayList;
import java.util.HashMap;

import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider.IStyledLabelProvider;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.StyledString;
import org.eclipse.jface.viewers.StyledString.Styler;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.etri.eDrone.Global;
import org.etri.eDrone.StylerProvider;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;

/*
 * 
 * 두번째 페이지 의존 정보 관리에 프로젝트 테이블을 구현한다.
 * 
 * 
 *  SharedVariables에 static으로 등록되며
 *	SharedVariables.getTVDependent();로 어느 클래스에서도 가져올 수 있다.
 * 
 * Treeviewer에서
 * SharedVariables.getTVDependet().setInput(LMMDependentProject);
 * 
 * 
 * Listviewer에서
 * SharedVariables.getTVDependet().setInput(LMMDependentProject);
 * 
 * 호출하는 부분을 찾으면 이 클래스의 행위를 더 잘 이해할 수 있다.
 * 
 */

public class Table_Dependencies {

	TableViewer viewer;

	public Table_Dependencies(Composite parent) {
		viewer = new TableViewer(parent, SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.FULL_SELECTION | SWT.BORDER);

		createColumns(viewer);
		final Table table = viewer.getTable();

		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.heightHint = 650;
		gd.horizontalSpan = 4;
		table.setLayoutData(gd);
		table.setHeaderVisible(true);
		table.setLinesVisible(true);

//		viewer.setContentProvider(ArrayContentProvider.getInstance());
		viewer.setContentProvider(new ContentProvider_depend_list());
		Global.dialog_opened.tableviwer_dependencies = viewer;
		viewer.setInput(new Model_Service());

	}

	private class ContentProvider_depend_list implements IStructuredContentProvider {

		public Object[] getElements(Object input) {
			return ((Model_Service) input).depend_list.toArray();
		}

	}

	private void createColumns(TableViewer viewer) {
		String[] titles = { "Name", "Location", "Dependencies" };

		int[] bounds = { 200, 400, 200 };

		TableViewerColumn col = createTableViewerColumn(titles[0], bounds[0]);
		col.getColumn().setAlignment(SWT.CENTER);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base p = (Model_Base) element;
				if (p.isProject == false) {
					return p.parent.toString();
				}
				return p.toString();
			}

			@Override
			public Image getImage(Object element) {
				return Global.image_Project;

			}
		});

		col = createTableViewerColumn(titles[1], bounds[1]);
		col.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Model_Base p = (Model_Base) element;
				if (p.isProject == false) {
					return p.parent.location;
				}
				return p.location;
			}
		});

		col = createTableViewerColumn(titles[2], bounds[2]);
		col.setLabelProvider(new DelegatingStyledCellLabelProvider(new DependLabelProvider()));

	}

	private class DependLabelProvider extends ColumnLabelProvider implements IStyledLabelProvider {

		@Override
		public StyledString getStyledText(Object element) {

			StylerProvider stylerProvider = new StylerProvider();

			StyledString styledString = new StyledString("\n");

			Styler styler = stylerProvider.getStyler(null, Display.getCurrent().getSystemColor(SWT.COLOR_RED), null);
			Styler styler2 = stylerProvider.getStyler(null, Display.getCurrent().getSystemColor(SWT.COLOR_DARK_GREEN),
					null);
			Styler styler3 = stylerProvider.getStyler(null, Display.getCurrent().getSystemColor(SWT.COLOR_DARK_BLUE),
					null);
			if (element instanceof Model_Base) {
				Model_Base mv = (Model_Base) element;
				HashMap<String, ArrayList<String>> hash_depend = mv.get_dpendencies();

				String[] arr = { "depend", "build_depend", "run_depend" };

				for (String str : arr) {
					ArrayList<String> list = hash_depend.get(str);
					if (list.size() < 1) {
						continue;
					}
					styledString.append("[" + str + " ]\n\n", styler2);
					for (String depend : list) {
						if (Global.list_pacakges.contains(depend) == true) {
							styledString.append("         " + depend + "\n");
						} else {
							styledString.append("         " + depend + "  (not found)" + "\n", styler);

						}

					}
					styledString.append("\n");
				}

			}
			styledString.append("\n- - - - - - - - - - - - - - - - - - - - -", styler3);
			return styledString;
		}

	}

	private TableViewerColumn createTableViewerColumn(String title, int bound) {

		final TableViewerColumn viewerColumn = new TableViewerColumn(viewer, SWT.NONE);
		final TableColumn column = viewerColumn.getColumn();
		column.setText(title);
		column.setWidth(bound);
		column.setResizable(true);
		column.setMoveable(true);
		return viewerColumn;

	}

}
