# eDrone_cmakelist_template

PRJ : projectName
DEP : dependencies


======================================================================================================================
<launch>  
<!-- connection setup -->  

	<arg name="fcu_url" default="udp://:14540@129.254.189.194:14557" />  
	<arg name="gcs_url" default="" />  
	<!-- <arg name="gcs_url" default="udp://@127.0.0.1:14550" /> -->  
	<arg name="tgt_system" default="1" />  
	<arg name="tgt_component" default="1" />  
	<arg name="my_args" default="" />
			
			
	<!-- nodes list -->  
					  
	<node name="mavros" pkg="mavros" type="mavros_node" output="screen">  
             <param name="fcu_url" value="$(arg fcu_url)" />  
	     <param name="gcs_url" value="$(arg gcs_url)" />  
	     <param name="target_system_id" value="$(arg tgt_system)" />  
	     <param name="target_component_id" value="$(arg tgt_component)" />  
	  
	     <!--rosparam command="load" file="$(find mavros)/launch/px4_blacklist.yaml"-->  
	  
	     <!-- enable heartbeat send and reduce timeout -->  
	     <param name="conn_heartbeat" value="5.0" />  
	     <param name="conn_timeout" value="5.0" />  
	     <!-- automatically start mavlink on USB -->  
	     <param name="startup_px4_usb_quirk" value="true" />  
	     <param name="mocap/use_tf" value="true"/>  
	     <param name="mocap/use_pose" value="false"/>  
	</node>  
					  
	<node name="eDrone_monitor_node" pkg="eDrone_lib" type="eDrone_monitor_node" output="screen"></node>    

	<node name="eDrone_control_node" pkg="eDrone_lib" type="eDrone_control_node" output="screen"></node>

	<node name="eDrone_autoflight_node" pkg="eDrone_lib" type="eDrone_autoflight_node" output="screen"></node> 

	<node name="eDrone_safety_node" pkg="eDrone_lib" type="eDrone_safety_node" output="screen"></node>     
	 
	<node name ="ex_?[PRJ]" pkg="?[PRJ]" type="ex_?[PRJ]" args="$(arg my_args)" output="screen"> </node>
	
</launch>

