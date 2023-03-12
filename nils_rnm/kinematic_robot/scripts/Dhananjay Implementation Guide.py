#Dhananjay Implementation Guide


# 1. Put this at the Beginning of your init
# Start and Finish Flags
self.task_start_flag = False
self.task_start_command = "model_registration_start"
self.task_finished_command = "model_registration_finished"
self.at_desired_goal_pose = False
self.joint_list_path    = os.path.join(os.path.dirname(__file__),'CV_camera_calibration_data/dataset001/collected_joint_list2.npy')  




# 2. Create Publisher and Subscriber in Init for task start command
# Initialize ROS Specific Functions
self.sub_task_command       = rospy.Subscriber('/task_command', String, self.callback_task_command)
self.pub_task_finished      = rospy.Publisher('/task_finished', String, queue_size=1)
self.pub_needle_goal_pose   = rospy.Publisher('/needle_goal_pose', Float64MultiArray, queue_size=1)
self.pub_goal_pose_js       = rospy.Publisher("/goal_pose_js", Float64MultiArray, queue_size=1)
self.sub_goal_pose_reached  = rospy.Subscriber("/goal_pose_reached", Int16, self.callback_goal_pose_reached)



# 3. Put Callback Function into Class Functions
def callback_task_command(self, msg : String):
    ''' Listening to task_command topic and checks for task_start_command'''
    if msg.data == self.task_start_command:
        self.task_start_flag = True
    else:
        self.task_start_flag = False



# 4. Put Function in Class to check if needle topic ist published
def is_topic_published(self, topic_name : str):
    ''' Checks the rostopic list for the given topic 
    '''
    topics  = rospy.get_published_topics()
    topics  = [topics[i][0] for i in range(len(topics))]
    return (topic_name in topics)
    



# 5 Add this at the Beginning of you main function. It will Check if all topic are published
# and waits for the start message
# Wait until process_manager is ready
rospy.logwarn('[MR] Waiting for PM...')
while not self.is_topic_published('/needle_goal_pose'): pass
rospy.logwarn('[MR] Topics from PM detected, waiting for task_start_command')
while not self.task_start_flag: pass
rospy.logwarn('[MR] Received task_start_command')
time.sleep(2)



# 6 Put this at the end of your main process, when everything is done, to let PM know you are finished
self.pub_task_finished.publish(self.task_finished_command)
self.pub_needle_goal_pose.publish(self.needle_goal_pose)
rospy.logwarn('[HiE] Main process finished')







# Add this to publish pose you want to take picture from and communicate with PM


# 1. Add this to your class function to publish a goal pose and append an index you check with the next function

def publish_desired_goal_pose(self, pose_js : np.array, pose_id : int):
    ''' Publihes current desired goal pose and current pose id using the goal_pose_js_pub.
        Also sets the self.curr_pose_id to the id that is sent in the message.
        Parameters:
            pose_js (List): Current goal pose in joint space
            pose_id (int): Current goal pose ID
    '''
    pose_js = np.append(pose_js, pose_id)
    msg         = Float64MultiArray()
    msg.data    = pose_js

    self.pub_goal_pose_js.publish(msg)
    

    rospy.logwarn(f"[CC] Send new goal_pose_js with ID {pose_id}")



# 2. add this to your class function to check if goal pose was reached and compare the index
def callback_goal_pose_reached(self, msg : Int16):
    ''' Checks the received confirmation ID and sets at_desired_goal_pose flag accordingly
    '''
    received_id = msg.data
    if received_id == self.joint_list_pos:
        rospy.logwarn(f"[CC] Received correct confirmation ID {received_id}")
        self.at_desired_goal_pose = True


# 3. Read in after Start command in main
    
    joint_list = np.load(self.joint_list_path)




# 4. Explanation how I solved the publish new goal pose stuff

#4.1
#  While Loop To collect data
# First of all I check in the while loop if I am still in the length of joint list I want to reach. If I already
# collected all data, I skip my while

#4.2
# When I still need to reached pose (while loop true) I check if the kinematics confirmed that I reached my desired goal pose
# with self.at_disred_goal_pose
# if so, I just to the stuff I want to do at this pose.

#4.3
#After doing all the stuff I reset the self.at.disired_goal_pose to false again, and increament my joint list, to prepare it for the next pose
# Then I check if I still have joints to publish (exceeded joint list?) and publish a new goal pose
while self.joint_list_pos < len(joint_list):

    if self.at_desired_goal_pose:
        self.collect_calibration_data()
        # Prepare For Next Goal Position
        self.at_desired_goal_pose = False

        self.joint_list_pos += 1
        
        if self.joint_list_pos < len(joint_list):
            self.publish_desired_goal_pose(joint_list[self.joint_list_pos], self.joint_list_pos)