package org.etri.eDroneLauncher;

import java.io.IOException;

import org.eclipse.core.commands.AbstractHandler;
import org.eclipse.core.commands.ExecutionEvent;
import org.eclipse.core.commands.ExecutionException;
import org.etri.eDrone.Global;

/**
 * Our sample handler extends AbstractHandler, an IHandler base class.
 * 
 * @see org.eclipse.core.commands.IHandler
 * @see org.eclipse.core.commands.AbstractHandler
 */
public class TerminateHandler extends AbstractHandler {

	@Override
	public Object execute(ExecutionEvent event) throws ExecutionException {
		String project_name = Global.CURRENT_TARGET_PROJECTNAME;
		int i = 0;
//		for (Process process : LaunchConfiguration.array_process) {
//			if (process != null && process.isAlive()) {
//				System.out.println("Process " + i + " terminated");
//				process.destroy();
//			}
//		}
		String[] comss = { "killall -9 eDrone_control_node", "killall -9 eDrone_monitor_node",
				"killall -9 eDrone_autoflight_node", "killall -9 eDrone_safety_node", "killall -9 mavros_node",
				"killall -9 ex_" + project_name };

		for (String com : comss) {
			try {
				runCmd(com.split(" "));
			} catch (IOException e) {
			}
		}
		if (Global.CURRENT_TARGET_ADDRESS.length() > 0 && Global.CURRENT_TARGET_ADDRESS.length() > 0
				&& Global.CURRENT_TARGET_PROJECTNAME.length() > 0 && Global.CURRENT_TARGET_USERNAME.length() > 0
				&& Global.CURRENT_TARGET_PASSWORD.length() > 0) {

			String target_port = Global.CURRENT_TARGET_PORT;
			String target_address = Global.CURRENT_TARGET_ADDRESS;
			String target_user = Global.CURRENT_TARGET_USERNAME;
			String target_password = Global.CURRENT_TARGET_PASSWORD;

			StringBuilder sb4 = new StringBuilder();
			sb4.append("sshpass ");
			sb4.append("-p " + target_password + " ");
			sb4.append("ssh -t -p ");
			sb4.append(target_port + " ");
			sb4.append(target_user + "@" + target_address + " ");

			String temp_command2[] = sb4.toString().split(" ");

			String[] kill_command = new String[temp_command2.length + 1];

			System.arraycopy(temp_command2, 0, kill_command, 0, temp_command2.length);

			String[] coms = { "killall -9 eDrone_control_node", "killall -9 eDrone_monitor_node",
					"killall -9 eDrone_autoflight_node", "killall -9 eDrone_safety_node", "killall -9 mavros_node",
					"killall -9 ex_" + project_name };

			for (String comm : coms) {

				kill_command[temp_command2.length] = comm;

				for (int j = 0; j < kill_command.length - 1; j++) {
					System.out.print(kill_command[j] + " ");
				}
				System.out.println(kill_command[kill_command.length - 1]);

				try {
					runCmd(kill_command);
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		return null;

	}

	private static void runCmd(String[] cmd) throws IOException {
		Runtime.getRuntime().exec(cmd);
	}
}
