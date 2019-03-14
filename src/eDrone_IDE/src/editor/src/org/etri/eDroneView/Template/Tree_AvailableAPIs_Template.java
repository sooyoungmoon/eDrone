package org.etri.eDroneView.Template;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IFolder;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.jface.resource.JFaceResources;
import org.eclipse.jface.resource.LocalResourceManager;
import org.eclipse.jface.resource.ResourceManager;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider;
import org.eclipse.jface.viewers.DelegatingStyledCellLabelProvider.IStyledLabelProvider;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.StyledString;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.TreeViewerColumn;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.IWorkbenchPage;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.dialogs.FilteredTree;
import org.eclipse.ui.ide.IDE;
import org.etri.eDrone.APIPatternFilter;
import org.etri.eDrone.Global;
import org.etri.eDroneModel.Model_Base;

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

public class Tree_AvailableAPIs_Template {

	public TreeViewer viewer;

	private class MyContentProvider implements ITreeContentProvider {

		@Override
		public Object[] getElements(Object inputElement) {

			Model_Base m = (Model_Base) inputElement;
			if (m.isAPI)
				return null;

			List<Model_Base> LNew = new ArrayList<Model_Base>();

			for (Model_Base entry : m.child) {

				if (entry.child.size() > 0 || entry.isTemplate || entry.isAPI) {

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

	public Tree_AvailableAPIs_Template(Composite parent) {

		APIPatternFilter filter = new APIPatternFilter();

		FilteredTree tree = new FilteredTree(parent, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL, filter,
				true);
		viewer = tree.getViewer();
		viewer.setLabelProvider(new LabelProvider());
		viewer.setContentProvider(new MyContentProvider());
		viewer.getTree().setHeaderVisible(false);
		viewer.getTree().setLinesVisible(true);

		GridData gd = new GridData(GridData.FILL_BOTH);
		gd.heightHint = 150;
		gd.horizontalSpan = 2;
		tree.setLayoutData(gd);

		TreeViewerColumn nameColumn = new TreeViewerColumn(viewer, SWT.NONE);
		nameColumn.getColumn().setWidth(150);
		nameColumn.getColumn().setText("API LIST");
		nameColumn.setLabelProvider(new DelegatingStyledCellLabelProvider(new NameLabelProvider()));

		viewer.addDoubleClickListener(new IDoubleClickListener() {

			@Override
			public void doubleClick(DoubleClickEvent event) {
				IStructuredSelection is = viewer.getStructuredSelection();

				Model_Base model = (Model_Base) is.getFirstElement();

				if (model.isAPI == false) {
					return;
				}

				IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();

				IProject project = root.getProject("[eDrone Template Edit Support]");
				try {
					if (project.exists() == false) {
						project.create(null);
						project.open(null);
					}
				} catch (CoreException e1) {
					e1.printStackTrace();
				}

				IFolder folder = project.getFolder("source");
				try {
					if (folder.exists() == false) {
						folder.create(false, true, null);
					}
				} catch (CoreException e) {
					e.printStackTrace();
				}

				String tplPath = Global.CATKIN_WORKSPACE_PATH + "/" + Global.DEPNEDING_FILES_PATH + "/template/source/"
						+ model.name + ".tpl";

				File f = new File(tplPath);

				if (f.exists() == false) {

					try {
						f.createNewFile();
					} catch (IOException e) {
						e.printStackTrace();
					}

				}
				IFile file = folder.getFile(model.name + ".tpl");
				try {
					if (file.exists() == false) {
						file.createLink(new URI(tplPath), IResource.BACKGROUND_REFRESH, null);
					}

					IWorkbenchPage page = PlatformUI.getWorkbench().getActiveWorkbenchWindow().getActivePage();

					if (page != null) {
						try {
							IDE.openEditor(page, file, true);

						} catch (PartInitException e) {
							e.printStackTrace();
						}
					}
				} catch (CoreException | URISyntaxException e) {
					e.printStackTrace();
				}
			}

		});

	}

	class NameLabelProvider extends LabelProvider implements IStyledLabelProvider {
		private ResourceManager resourceManager;

		@Override
		public StyledString getStyledText(Object element) {
			if (element instanceof Model_Base) {
				Model_Base mv = (Model_Base) element;
				StyledString styledString = new StyledString(mv.name);
				List<Model_Base> child = mv.child;
				if (child != null && !mv.isAPI && !mv.isRoot && !mv.isTemplate) {
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
					if (((Model_Base) element).name.equals("Global")) {
						return Global.image_Global;
					}
					return Global.image_API;
				} else if (((Model_Base) element).isTemplate) {
					return Global.image_Anchor;
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
			if (resourceManager != null) {
				resourceManager.dispose();
				resourceManager = null;
			}
		}

	}

}
