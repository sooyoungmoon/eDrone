package org.etri.eDroneWizard;

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
 */

public class Tree_AvailableAPIs {

	public Tree_AvailableAPIs(Composite parent) {

		APIPatternFilter filter = new APIPatternFilter();

		FilteredTree tree = new FilteredTree(parent, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL, filter,
				true);
		TreeViewer viewer = tree.getViewer();

		viewer.setLabelProvider(new LabelProvider());
		viewer.setContentProvider(new MyContentProvider());
		viewer.getTree().setHeaderVisible(false);
		viewer.getTree().setLinesVisible(true);

		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.grabExcessHorizontalSpace = true;
//		gd.heightHint = 330;
//		gd.widthHint = 100;
//		gd.verticalSpan = 3;
		gd.heightHint = 280;
		gd.widthHint = 100;
		gd.verticalSpan = 2;
		tree.setLayoutData(gd);

		TreeViewerColumn viewerColumn = new TreeViewerColumn(viewer, SWT.NONE);
		viewerColumn.getColumn().setWidth(100);
		viewerColumn.setLabelProvider(new DelegatingStyledCellLabelProvider(new MyViewLabelProvider()));

		viewer.addDoubleClickListener(new IDoubleClickListener() {
			@Override
			public void doubleClick(DoubleClickEvent event) {
				if (!event.getSelection().isEmpty()) {

					IStructuredSelection thisSelection = (IStructuredSelection) event.getSelection();
					if (thisSelection == null)
						return;
					Object selectedNode = thisSelection.getFirstElement();
					Model_Base model_old = (Model_Base) selectedNode;

					if (model_old.isAPI) {

						Model_Base model = new Model_Base(model_old);

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
						Model_Service model_service = (Model_Service) Global.wizard_page2.tableviwer_dependencies
								.getInput();

						for (Model_Base m : model_service.depend_list) {
							if (model.parent.name.equals(m.name)) {
								return;
							}
						}
						model_service.depend_list.add(model.parent);
						model.parent.get_dpendencies();
						Global.wizard_page2.tableviwer_dependencies.setInput(model_service);

//						Integer last_index = Global.wizard_page2.tableviewer_selected_api.getTable().getItemCount() - 1;

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

		Global.wizard_page2.treeviewer_availabe_api = viewer;
	}

	class MyViewLabelProvider extends LabelProvider implements IStyledLabelProvider {
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

}
