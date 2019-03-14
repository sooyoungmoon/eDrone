package org.etri.eDroneView.Service;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.jface.resource.JFaceResources;
import org.eclipse.jface.resource.LocalResourceManager;
import org.eclipse.jface.resource.ResourceManager;
import org.eclipse.jface.viewers.AbstractTreeViewer;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider.IStyledLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IContentProvider;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.StyledString;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.TreeViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.dialogs.FilteredTree;
import org.etri.eDrone.APIPatternFilter;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;
import org.etri.eDroneModel.Model_Service;

/*
 * 
 * 두번째 페이지 왼쪽 상단 트리뷰어를 구성한다.
 *
 * SharedVariables에 static으로 등록되며
 * SharedVariables.getTRVService();로 어느 클래스에서도 가져올 수 있다.
 * 
 * eDroneWizardPage.java에서  아래 함수를 호출해서 이 트리에 정보를 준다.
 * SharedVariables.getTRVService().setInput(MMRoot);
 * 
 */

public class Tree_AvailableAPIs_Edit {

	public TreeViewer viewer;

	public Composite parent;

	public Tree_AvailableAPIs_Edit(Composite parent) {

		this.parent = parent;

		APIPatternFilter filter = new APIPatternFilter();

		FilteredTree tree = new FilteredTree(parent, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL, filter,
				true);
		viewer = tree.getViewer();
		viewer.setLabelProvider(new LabelProvider());
		viewer.setContentProvider(new MyContentProvider());
		viewer.getTree().setHeaderVisible(false);
		viewer.getTree().setLinesVisible(true);

		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.heightHint = 180;
		gd.widthHint = 100;
		gd.verticalSpan = 2;
		tree.setLayoutData(gd);

		TreeViewerColumn nameColumn = new TreeViewerColumn(viewer, SWT.NONE);
		nameColumn.getColumn().setWidth(150);
		nameColumn.getColumn().setText("API LIST");
		nameColumn.setLabelProvider(new DelegatingStyledCellLabelProvider(new NameLabelProvider()));
		viewer.setInput(Global.MODEL_API_ROOT);
		viewer.expandAll();

		viewer.addDoubleClickListener(new IDoubleClickListener() {
			@Override
			public void doubleClick(DoubleClickEvent event) {
				if (!event.getSelection().isEmpty()) {

					IStructuredSelection is = (IStructuredSelection) event.getSelection();
					if (is == null)
						return;
					Model_Base model_old = (Model_Base) is.getFirstElement();

					if (model_old.isAPI) {

						Model_Base model = new Model_Base(model_old);
						List<Model_Base> list = null;
						Model_Service ms = (Model_Service) Global.dialog_opened.tableviewer_selected_api.getInput();

						list = ms.api_list;

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

						Global.dialog_opened.tableviewer_selected_api.setInput(ms);

						Global.dialog_opened.tableviewer_selected_api.setSelection(new StructuredSelection(model),
								true);

						Model_Service model_service = (Model_Service) Global.dialog_opened.tableviwer_dependencies
								.getInput();

						Model_Base pm = get_parent(model);
						if (model_service.depend_list.size() > 0) {
							for (Model_Base m : model_service.depend_list) {

								if (pm == null)
									return;

								if (pm.name.equals(m.name) == true) {
									return;
								}
							}
						}
						model_service.depend_list.add(pm);
						model.parent.get_dpendencies();
						Global.dialog_opened.tableviwer_dependencies.setInput(model_service);

						Global.validate_service(false);

					} else {
						TreeViewer treeViewer = (TreeViewer) event.getViewer();
						IContentProvider provider = treeViewer.getContentProvider();

						if (provider instanceof ITreeContentProvider) {
							ITreeContentProvider treeProvider = (ITreeContentProvider) provider;

							if (!treeProvider.hasChildren(model_old))
								return;

							if (treeViewer.getExpandedState(model_old))
								treeViewer.collapseToLevel(model_old, AbstractTreeViewer.ALL_LEVELS);
							else
								treeViewer.expandToLevel(model_old, 1);
						}
					}
				}
			}
		});
		Global.dialog_opened.treeviewer_availabe_api = viewer;
	}

	private Model_Base get_parent(Model_Base model_new) {

		if (model_new.name.equals("root")) {
			return null;
		}
		if (model_new.isProject) {
			return model_new;
		} else {
			return get_parent(model_new.parent);
		}

	}

	private class MyContentProvider implements ITreeContentProvider {

		@Override
		public Object[] getElements(Object inputElement) {

			Model_Base m = (Model_Base) inputElement;
			if (m.isAPI)
				return null;

			List<Model_Base> LNew = new ArrayList<Model_Base>();

			for (Model_Base entry : m.child) {

				if (entry.child.size() > 0) {

					LNew.add(entry);

				}
			}
			return LNew.toArray();
		}

		@Override
		public Object[] getChildren(Object parentElement) {
			return getElements(parentElement);
		}

		@Override
		public Object getParent(Object element) {
			if (element == null) {
				return null;
			}

			return ((Model_Base) element).getParent();
		}

		@Override
		public boolean hasChildren(Object element) {
			return ((Model_Base) element).child.size() > 0 && ((Model_Base) element).isAPI == false;
		}

	}

	class NameLabelProvider extends LabelProvider implements IStyledLabelProvider {
		private ResourceManager resourceManager;

		@Override
		public StyledString getStyledText(Object element) {
			if (element instanceof Model_Base) {
				Model_Base mv = (Model_Base) element;
				StyledString styledString = new StyledString(mv.name);
				List<Model_Base> child = mv.child;
				if (child != null && !mv.isAPI && !mv.isRoot) {
					styledString.append(" ( " + child.size() + " ) ", StyledString.COUNTER_STYLER);
				}
				return styledString;
			}
			return null;
		}

		protected ResourceManager getResourceManager() {
			if (resourceManager == null) {
				resourceManager = new LocalResourceManager(JFaceResources.getResources());
			}
			return resourceManager;
		}

		@Override
		public Image getImage(Object element) {
			if (element instanceof Model_Base) {
				if (((Model_Base) element).isProject) {
					return Global.image_Project;
				} else if (((Model_Base) element).isAPI) {
					return Global.image_API;
				}
			}
			return Global.image_Folder;
		}

		@Override
		public void dispose() {
			// garbage collect system resources
			if (resourceManager != null) {
				resourceManager.dispose();
				resourceManager = null;
			}
		}

	}

	class HasTemplateLabelProvider extends LabelProvider implements IStyledLabelProvider {
		private ResourceManager resourceManager;

		@Override
		public StyledString getStyledText(Object element) {
//			if (element instanceof MODEL) {
//				MODEL mv = (MODEL) element;
//				StyledString styledString = new StyledString(mv.name);
//
//				return styledString;
//			}
			return new StyledString("");
		}

		protected ResourceManager getResourceManager() {
			if (resourceManager == null) {
				resourceManager = new LocalResourceManager(JFaceResources.getResources());
			}
			return resourceManager;
		}

		@Override
		public Image getImage(Object element) {
			if (element instanceof Model_Base) {
				if (((Model_Base) element).hasTemplate) {
					return Global.image_Accepted;
				}
			}
			return null;
		}

		@Override
		public void dispose() {
			// garbage collect system resources
			if (resourceManager != null) {
				resourceManager.dispose();
				resourceManager = null;
			}
		}

	}

}
