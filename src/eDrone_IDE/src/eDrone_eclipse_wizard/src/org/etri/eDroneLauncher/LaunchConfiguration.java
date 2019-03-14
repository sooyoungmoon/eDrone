package org.etri.eDroneLauncher;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;

import org.eclipse.core.resources.IProject;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.debug.core.ILaunch;
import org.eclipse.debug.core.ILaunchConfiguration;
import org.eclipse.debug.core.model.LaunchConfigurationDelegate;
import org.eclipse.ui.console.ConsolePlugin;
import org.eclipse.ui.console.IConsole;
import org.eclipse.ui.console.IConsoleManager;
import org.eclipse.ui.console.MessageConsole;
import org.eclipse.ui.console.MessageConsoleStream;
import org.etri.eDrone.Global;

public class LaunchConfiguration extends LaunchConfigurationDelegate {

	public static Process array_process[];
	public static int index;
	private String taget_device;
	private String target_address;
	private String target_port;
	private String target_user;
	private String target_password;
	private String target_workspace_path;
	public static String project_name;
	private String project_location;
	private String project_arguments;
	private String is_log;
	private String is_run_only;
	private String is_cross;

	@SuppressWarnings("unused")
	private boolean isBuildSuccess;

	private MessageConsole findConsole(String name) {
		ConsolePlugin plugin = ConsolePlugin.getDefault();
		IConsoleManager conMan = plugin.getConsoleManager();
		IConsole[] existing = conMan.getConsoles();
		for (int i = 0; i < existing.length; i++) {
			if (name.equals(existing[i].getName())) {
				conMan.showConsoleView(existing[i]);
				return (MessageConsole) existing[i];
			}
		}
		// no console found, so create a new one
		MessageConsole myConsole = new MessageConsole(name, null);
		conMan.addConsoles(new IConsole[] { myConsole });
		conMan.showConsoleView(myConsole);
		return myConsole;
	}

	@Override
	public boolean buildForLaunch(ILaunchConfiguration configuration, String mode, IProgressMonitor monitor)
			throws CoreException {

		array_process = new Process[30];
		index = 0;
		isBuildSuccess = true;
		taget_device = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_DEVICE, "");
		target_address = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_ADDRESS, "");
		target_port = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_PORT, "");
		target_user = configuration.getAttribute(LaunchConfigurationAttributes.USER, "");
		target_password = configuration.getAttribute(LaunchConfigurationAttributes.PASSWORD, "");
		target_workspace_path = configuration.getAttribute(LaunchConfigurationAttributes.CATKIN_WS_PATH, "");
		project_name = configuration.getAttribute(LaunchConfigurationAttributes.PROJECT_NAME, "");
		project_location = configuration.getAttribute(LaunchConfigurationAttributes.PROJECT_LOCATION, "");
		project_arguments = configuration.getAttribute(LaunchConfigurationAttributes.ARGUMENTS, "");
		is_log = configuration.getAttribute(LaunchConfigurationAttributes.ISLOG, "");
		is_run_only = configuration.getAttribute(LaunchConfigurationAttributes.ISRUNONLY, "NO");
		is_cross = configuration.getAttribute(LaunchConfigurationAttributes.ISCROSS, "");

		MessageConsole myConsole = findConsole("Console");
		MessageConsoleStream stream = myConsole.newMessageStream();
		PrintStream myS = new PrintStream(stream);
		System.setOut(myS);
		System.setErr(myS);
		System.out.println("TARGET_DEVICE :" + taget_device);
		System.out.println("TARGET_ADDRESS :" + target_address);
		System.out.println("TARGET_PORT :" + target_port);
		System.out.println("PASSWORD :" + target_password);
		System.out.println("TARGET_CATKIN_WS_PATH :" + target_workspace_path);
		System.out.println("--------------");
		System.out.println("PROJECT_NAME :" + project_name);
		System.out.println("PROJECT_LOCATION :" + project_location);
		System.out.println("ARGUMENTS :" + project_arguments);
		System.out.println("\n\n\n>>>>>>  CATKIN_BUILD START " + project_name);
		System.out.println("\n\n\n");

		if (is_run_only.equals("NO")) {
			if (taget_device.equals("odroid")) {
				is_cross = configuration.getAttribute(LaunchConfigurationAttributes.ISCROSS, "NO");
				if (is_cross.equals("YES")) {

					// 먼저 타겟에 복사
//				StringBuilder sb_src = new StringBuilder();
//				sb_src.append("sshpass ");
//				if (target_password.equals("") == false) {
//
//					sb_src.append("-p " + target_password + " ");
//				} else {
//
//					isBuildSuccess = false;
//					return false;
//				}
//				sb_src.append("scp -r -P ");
//				sb_src.append(target_port + " ");
//				sb_src.append(project_location + "/src/" + project_name + " ");
//				sb_src.append(target_user + "@" + target_address + ":" + target_workspace_path + "/src/");
//				System.out.println(sb_src.toString());
//				String copy_src_command[] = sb_src.toString().split(" ");
//				try {
//
//					runCmd(copy_src_command, true, null, false);
//					System.out.println(">> 소스가 타겟보드로 복사되었습니다.");
//
//				} catch (IOException e) {
//					isBuildSuccess = false;
//					e.printStackTrace();
//					return false;
//				}

					File f = new File(Global.CATKIN_WORKSPACE_PATH);
					File f2 = new File(project_location);

					if (f.getAbsolutePath().equals(f2.getAbsolutePath()) == false) {
						String cmd_copy_proj[] = { "cp", "-rf", Global.CATKIN_WORKSPACE_PATH + "/src/" + project_name,
								project_location + "/src" };
						try {
							runCmd(cmd_copy_proj, false, null, false);

						} catch (IOException e) {
							isBuildSuccess = false;
							e.printStackTrace();
						}
					}

					try {
						String cmd = "sshpass -p edrone ssh -t root@localhost";
						String temp_command[] = cmd.split(" ");
						String[] run_command = new String[temp_command.length + 1];
						System.arraycopy(temp_command, 0, run_command, 0, temp_command.length);
						String com = "cd " + project_location + " && catkin config && catkin build " + project_name;
						run_command[temp_command.length] = com;

						for (int i = 0; i < run_command.length - 1; i++) {
							System.out.print(run_command[i] + " ");
						}
						System.out.println(run_command[run_command.length - 1]);

						runCmd(run_command, true, null, true);

					} catch (IOException e1) {
						e1.printStackTrace();
						System.out.println("catkin build fail");
						isBuildSuccess = false;
						return false;
					}

					String home = System.getenv("HOME");
					String deb_path = home + "/debian/";
					mkdirs(deb_path);
					mkdirs(deb_path + project_name);
					mkdirs(deb_path + project_name + "/" + project_name);
					mkdirs(deb_path + project_name + "/" + project_name + "/DEBIAN");
					File f_contorl = mkfiles(deb_path + project_name + "/" + project_name + "/DEBIAN/control");
					File f_postinst = mkfiles(deb_path + project_name + "/" + project_name + "/DEBIAN/postinst");

					StringBuilder sb = new StringBuilder();
					sb.append("Package: " + project_name + "\n" + "Version: 0.0.1\n" + "Section: devel\n"
							+ "Priority: optional\n" + "Architecture: all\n" + "Depends: cmake\n" + "Recommends: \n"
							+ "Maintainer: Developers <moonmous@gmail.com>\n" + "Homepage: www\n"
							+ "Description: A test (ROS) package for distribution\n");

					try {
						FileWriter in = new FileWriter(f_contorl);
						in.write(sb.toString());
						in.close();
					} catch (IOException e) {
						e.printStackTrace();
					}

					try {
						FileWriter in = new FileWriter(f_postinst);
						in.write("ldconfig /opt/ros/kinetic");
						in.close();
					} catch (IOException e) {
						e.printStackTrace();
					}

					String mode_change_01[] = { "chmod", "755", f_postinst.getAbsolutePath() };

					String target_path = deb_path + project_name + "/" + project_name + "/opt/ros/kinetic/";
					String cmd_make_dir_01[] = { "mkdir", "-p", target_path + "include/" + project_name };
					String cmd_make_dir_02[] = { "mkdir", "-p", target_path + "lib/" + project_name };
					String cmd_make_dir_03[] = { "mkdir", "-p", target_path + "share/" };

					String cmd_copy_01[] = { "cp", "-rf", project_location + "/install/include/" + project_name,
							target_path + "include/" };
					String cmd_copy_02[] = { "cp", "-rf", project_location + "/install/lib/" + project_name,
							target_path + "lib" };
					String cmd_copy_03[] = { "cp", "-rf", project_location + "/install/share/" + project_name,
							target_path + "share" };

					try {
						runCmd(mode_change_01, false, null, false);
						runCmd(cmd_make_dir_01, false, null, false);
						runCmd(cmd_make_dir_02, false, null, false);
						runCmd(cmd_make_dir_03, false, null, false);

						runCmd(cmd_copy_01, false, null, false);
						runCmd(cmd_copy_02, false, null, false);
						runCmd(cmd_copy_03, false, null, false);
					} catch (IOException e) {
						isBuildSuccess = false;
						e.printStackTrace();
					}

					String cmd_make_debin[] = { "dpkg", "-b", project_name };

					try {
						runCmd(cmd_make_debin, true, deb_path + project_name, false);

					} catch (IOException e) {
						e.printStackTrace();
					}
					// Copy Command for project from the PC to the target board
					StringBuilder sb3 = new StringBuilder();
					sb3.append("sshpass ");
					if (target_password.equals("") == false) {

						sb3.append("-p " + target_password + " ");
					} else {

						isBuildSuccess = false;
						return false;
					}
					sb3.append("scp -r -P ");
					sb3.append(target_port + " ");
					sb3.append(deb_path + project_name + "/" + project_name + ".deb" + " ");
					sb3.append(target_user + "@" + target_address + ":" + this.target_workspace_path + "/devel/");
					System.out.println(sb3.toString());
					String copy_command[] = sb3.toString().split(" ");
					try {

						runCmd(copy_command, true, null, false);
						System.out.println(">> .deb 파일이 타겟보드로 복사되었습니다.");

					} catch (IOException e) {
						isBuildSuccess = false;
						e.printStackTrace();
						return false;
					}

					try {
						String cmd = "sshpass -p " + target_password + " ssh -t -p " + target_port + " root@"
								+ target_address;
						String temp_command[] = cmd.split(" ");
						String[] run_command = new String[temp_command.length + 1];
						System.arraycopy(temp_command, 0, run_command, 0, temp_command.length);
						String com = "cd " + target_workspace_path + "/devel && dpkg -i " + project_name + ".deb";
						run_command[temp_command.length] = com;

						for (int i = 0; i < run_command.length - 1; i++) {
							System.out.print(run_command[i] + " ");
						}
						System.out.println(run_command[run_command.length - 1]);

						runCmd(run_command, true, null, true);

					} catch (IOException e1) {
						e1.printStackTrace();
						isBuildSuccess = false;
						return false;
					}
				} else {

					StringBuilder sb = new StringBuilder();
					sb.append("sshpass ");
					if (target_password.equals("") == false) {

						sb.append("-p " + target_password + " ");
					} else {

						isBuildSuccess = false;
						return false;
					}
					sb.append("scp -r -P ");
					sb.append(target_port + " ");
					sb.append(project_location + "/src/" + project_name + " ");
					sb.append(target_user + "@" + target_address + ":" + target_workspace_path + "/src/");
					System.out.println(sb.toString());
					String copy_command[] = sb.toString().split(" ");

					try {

						runCmd(copy_command, true, null, false);
						System.out.println(">> Copy Complete");
					} catch (IOException e) {
						isBuildSuccess = false;
						e.printStackTrace();
					}

				}

				StringBuilder sb2 = new StringBuilder();
				sb2.append("sshpass:");
				if (target_password.equals("") == false) {

					sb2.append("-p:" + target_password + ":");
				} else {
					isBuildSuccess = false;
					return false;
				}
				sb2.append("ssh:-t:-p:");
				sb2.append(target_port + ":");
				sb2.append(target_user + "@" + target_address + ":");
				sb2.append("(cd " + target_workspace_path + " && " + "catkin build " + project_name + ")");
				System.out.println(sb2.toString().replace(":", " "));
				String build_command[] = sb2.toString().split("\\:");
				try {

					runCmd(build_command, true, null, false);
					System.out.println(">> Build Complete");
				} catch (IOException e) {
					isBuildSuccess = false;
					e.printStackTrace();
				}

			} else {
				String[] cmd = new String[3];
				cmd[0] = "catkin";
				cmd[1] = "build";
				cmd[2] = project_name;
				try {
					runCmd(cmd, true, Global.CATKIN_WORKSPACE_PATH, true);
				} catch (IOException e1) {
					e1.printStackTrace();
					System.out.println("catkin build fail");
					isBuildSuccess = false;
					return false;
				}

			}
		}
		return false;
	}

	private void myPrint(String line) {

		System.out.println("^^^^ " + line);
	}

	@Override
	protected void buildProjects(IProject[] projects, IProgressMonitor monitor) throws CoreException {
		System.out.println("buildproject");
		return;
	}

	@Override
	public void launch(ILaunchConfiguration configuration, String mode, ILaunch launch, IProgressMonitor monitor)
			throws CoreException {

		isBuildSuccess = true;
		taget_device = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_DEVICE, "odroid");
		target_address = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_ADDRESS, "localhost");
		target_port = configuration.getAttribute(LaunchConfigurationAttributes.TARGET_PORT, "22");
		target_user = configuration.getAttribute(LaunchConfigurationAttributes.USER, "");
		target_password = configuration.getAttribute(LaunchConfigurationAttributes.PASSWORD, "");
		target_workspace_path = configuration.getAttribute(LaunchConfigurationAttributes.CATKIN_WS_PATH,
				"/home/" + taget_device + "/catkin_ws");
		project_name = configuration.getAttribute(LaunchConfigurationAttributes.PROJECT_NAME, "");
		project_location = configuration.getAttribute(LaunchConfigurationAttributes.PROJECT_LOCATION, "sd");
		project_arguments = configuration.getAttribute(LaunchConfigurationAttributes.ARGUMENTS, "");
		is_log = configuration.getAttribute(LaunchConfigurationAttributes.ISLOG, "NO");
		is_run_only = configuration.getAttribute(LaunchConfigurationAttributes.ISRUNONLY, "NO");
		is_cross = configuration.getAttribute(LaunchConfigurationAttributes.ISCROSS, "NO");

		if (taget_device.equals("odroid")) {

			Global.CURRENT_TARGET_ADDRESS = target_address;
			Global.CURRENT_TARGET_PORT = target_port;
			Global.CURRENT_TARGET_PROJECTNAME = project_name;
			Global.CURRENT_TARGET_USERNAME = target_user;
			Global.CURRENT_TARGET_PASSWORD = target_password;
			// kill command
			StringBuilder sb4 = new StringBuilder();
			sb4.append("sshpass ");
			if (target_password.equals("") == false) {

				sb4.append("-p " + target_password + " ");
			} else {

				return;
			}
			sb4.append("ssh -t -p ");
			sb4.append(target_port + " ");
			sb4.append(target_user + "@" + target_address + " ");

			String temp_command2[] = sb4.toString().split(" ");

			String[] kill_command = new String[temp_command2.length + 1];

			System.arraycopy(temp_command2, 0, kill_command, 0, temp_command2.length);

			String com2 = "(killall -9 eDrone_control_node && killall -9 eDrone_monitor_node && killall -9 eDrone_autoflight_node && killall -9 eDrone_safety_node "
					+ "&& killall -9 mavros_node && killall -9 ex_" + project_name + ")";

			kill_command[temp_command2.length] = com2;

			for (int i = 0; i < kill_command.length - 1; i++) {
				System.out.print(kill_command[i] + " ");
			}
			System.out.println(kill_command[kill_command.length - 1]);

			// run command - NON cross compile

			StringBuilder sb3 = new StringBuilder();
			sb3.append("sshpass ");
			if (target_password.equals("") == false) {

				sb3.append("-p " + target_password + " ");
			} else {

				return;
			}
			sb3.append("ssh -t -p ");
			sb3.append(target_port + " ");
			sb3.append(target_user + "@" + target_address + " ");

			String temp_command[] = sb3.toString().split(" ");

			String[] run_command = new String[temp_command.length + 1];

			System.arraycopy(temp_command, 0, run_command, 0, temp_command.length);

			String com = "(cd " + target_workspace_path + " && source " + target_workspace_path
					+ "/devel/setup.bash && " + "roslaunch " + project_name + " " + project_name + ".launch" + " "
					+ project_arguments + ")";

			run_command[temp_command.length] = com;

			for (int i = 0; i < run_command.length - 1; i++) {
				System.out.print(run_command[i] + " ");
			}
			System.out.println(run_command[run_command.length - 1]);

			boolean isLogOn = false;
			if (is_log.equals("YES")) {
				isLogOn = true;
			}
			try {
				runCmd(kill_command, isLogOn, null, false);
				System.out.println(">> kill Command Sent");

				runCmd(run_command, isLogOn, null, true);
				System.out.println(">> Run Command Sent");

			} catch (IOException e) {
				e.printStackTrace();
			}

		} else {
			// x86_64

			Global.CURRENT_TARGET_ADDRESS = "";
			Global.CURRENT_TARGET_PORT = "";
			Global.CURRENT_TARGET_PROJECTNAME = project_name;
			Global.CURRENT_TARGET_USERNAME = "";
			Global.CURRENT_TARGET_PASSWORD = "";

			StringBuilder sb4 = new StringBuilder();
			sb4.append("sshpass ");
			if (target_password.equals("") == false) {
				sb4.append("-p " + target_password + " ");
			} else {

				return;
			}
			sb4.append("ssh -t -p ");
			sb4.append(target_port + " ");
			sb4.append(target_user + "@" + target_address + " ");

			String temp_command2[] = sb4.toString().split(" ");

			String[] kill_command = new String[temp_command2.length + 1];

			System.arraycopy(temp_command2, 0, kill_command, 0, temp_command2.length);

			String com2 = "(killall -9 eDrone_control_node && killall -9 eDrone_monitor_node && killall -9 eDrone_autoflight_node && killall -9 eDrone_safety_node "
					+ "&& killall -9 mavros_node && killall -9 ex_" + project_name + ")";

			kill_command[temp_command2.length] = com2;

			for (int i = 0; i < kill_command.length - 1; i++) {
				System.out.print(kill_command[i] + " ");
			}
			System.out.println(kill_command[kill_command.length - 1]);

			// run command
			StringBuilder sb3 = new StringBuilder();
			sb3.append("sshpass ");
			if (target_password.equals("") == false) {
				sb3.append("-p " + target_password + " ");
			} else {
				return;
			}
			sb3.append("ssh -t -p ");
			sb3.append(target_port + " ");
			sb3.append(target_user + "@" + target_address + " ");

			String temp_command[] = sb3.toString().split(" ");

			String[] run_command = new String[temp_command.length + 1];

			System.arraycopy(temp_command, 0, run_command, 0, temp_command.length);

			String com = "source " + project_location + "/devel/setup.bash && roslaunch " + project_name + " "
					+ project_name + ".launch " + project_arguments;

			run_command[temp_command.length] = com;

			for (int i = 0; i < run_command.length - 1; i++) {
				System.out.print(run_command[i] + " ");
			}
			System.out.println(run_command[run_command.length - 1]);

			boolean isLogOn = false;
			if (is_log.equals("YES")) {
				isLogOn = true;
			}
			try {
				runCmd(kill_command, isLogOn, null, false);
				System.out.println(">> kill Command Sent");

				runCmd(run_command, isLogOn, null, true);
				System.out.println(">> Run Command Sent");

			} catch (IOException e) {
				e.printStackTrace();
			}

		}

	}

	private static void runCmd(String[] cmd, boolean needLog, String path, boolean shouldHold) throws IOException {

		Process pb;
		if (path == null || path.length() < 1) {
			pb = Runtime.getRuntime().exec(cmd);

		} else {
			pb = Runtime.getRuntime().exec(cmd, null, new File(path));
		}

		if (shouldHold == true) {
			array_process[index] = pb;
			index++;
		}
		if (needLog == true) {
			String line;
			BufferedReader input = new BufferedReader(new InputStreamReader(pb.getInputStream()));
			while ((line = input.readLine()) != null) {
				System.out.println(line);
			}
			input.close();
		}
	}

	private File mkdirs(String path) {
		File desti = new File(path);
		myPrint(path);

		// 해당 디렉토리의 존재여부를 확인
		if (!desti.exists()) {
			// 없다면 생성
			desti.mkdirs();
			return desti;
		} else {
			// 있다면 현재 디렉토리 파일을 삭제
			File[] destroy = desti.listFiles();
			for (File des : destroy) {
				des.delete();
			}
			return null;
		}
	}

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

	public static void copyAll(File sourceF, File targetF) {

		File[] ff = sourceF.listFiles();
		for (File file : ff) {
			File temp = new File(targetF.getAbsolutePath() + File.separator + file.getName());
			if (file.isDirectory()) {
				temp.mkdir();
				copyAll(file, temp);
			} else {
				FileInputStream fis = null;
				FileOutputStream fos = null;
				try {
					fis = new FileInputStream(file);
					fos = new FileOutputStream(temp);
					byte[] b = new byte[4096];
					int cnt = 0;
					while ((cnt = fis.read(b)) != -1) {
						fos.write(b, 0, cnt);
					}
				} catch (Exception e) {
					e.printStackTrace();
				} finally {
					try {
						fis.close();
						fos.close();
					} catch (IOException e) {
						e.printStackTrace();
					}

				}
			}
		}
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
