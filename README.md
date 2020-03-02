# Minion_Payload_Transportation
Mobile robots installed with linear piston on it's top trying to keep the payload horizontal while moving through uneven surface. 

![ezgif com-video-to-gif](https://user-images.githubusercontent.com/31062159/75655170-4f5c2280-5c87-11ea-8664-3cce4e6768b7.gif)

Instructions:-
1. In 1st terminal : "roslaunch minion_robot gazebo.launch"

	It will start gazebo with customized simple world from minion_world.world. You can choose heightmap.world by modifying gazebo.launch file.
  
2. If you have choosen minion_world.launch then, in 2nd terminal: "roslaunch minion_robot arena.launch"

	Two type of arena are available. Choose by commenting/uncommenting in arena.launch file
  
3. In 3rd  terminal : "roslaunch minion_robot robots.launch"

	It will launch the number of minion robots and their originating coordinates.
  
4. In 4th   terminal : "roslaunch minion_robot payload.launch"

	Payload will be placed on the robots.
  
5. In 5th terminal: "rosrun minion_robot multi.py"

	All robot will start moving and piston will move up/down to maintain the 	horizontal position 	of payload. It will suscribe the payload orientation 	information from 	“ns/gazebo/model_states” and  publish the piston position 	to “ns/effort_controllers/	command” 
  
 Note: 
 
    • <max_contacts>200</max_contacts> in _.world file. Default value is 20, but increased for frictional force contact between payload and piston plate.
    
    • <real_time_update_rate>25</real_time_update_rate> in _.world file. Default is 100, which slides the robots by vibrations. Gazebo limitation.
    
    • In minion.urdf a spherical ball joint is made between piston plate and rod’s end by placing 3 mutual perpendicular revolute joints. (Junta1, Junta2, Jumta3)
    
    • For the piston, type: "joint_state_controller/JointStateController" and "<hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>"  is used.
