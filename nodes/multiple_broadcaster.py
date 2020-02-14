#!/usr/bin/env python  
import rospy
import time
import tf
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
# from turtle_tf_3d.get_model_gazebo_pose import GazeboModel

def handle_turtle_pose(pose_msg, robot_name):
    br = tf.TransformBroadcaster()

    br.sendTransform((pose_msg.position.x,pose_msg.position.y, pose_msg.position.z),
                    (pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w),
                    rospy.Time.now(),
                    robot_name,
                    "/world")

def publisher_of_tf():

    # rospy.init_node('publisher_of_tf_node', anonymous=True)
    # robot_name_list = ["Robot1", "Robot2"]
    # gazebo_model_object = GazeboModel(robot_name_list)

    # for robot_name in robot_name_list:
    #     pose_now = gazebo_model_object.get_model_gazebo_pose(robot_name)

    #     time.sleep(1)
    #     rospy.loginfo("Ready.. Starting to Publish TF data now...")

    #     rate = rospy.Rate(5)
    #     while not rospy.is_shutdown():
    #         for robot_name in robot_name_list:
    #             pose_now = gazebo_model_object.get_model_gazebo_pose(robot_name)
    #             if not pose_now:
    #                 print "The Pose is not yet"+str(robot_name)+" available... Please try again later"
    #             else:
    #                 handle_turtle_pose(pose_now, robot_name)
    #         rate.sleep()
    
    rospy.init_node('publisher_of_tf_node', anonymous=True)
    robot_name_list = ["Robot1", "Robot2"]
    gazebo_model_object = GazeboModel(robot_name_list)
    time.sleep(1)
    rospy.loginfo("Ready.. Starting to Publish TF data now...")

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        for robot_name in robot_name_list:
            pose_now = gazebo_model_object.get_model_gazebo_pose(robot_name)
            if not pose_now:
                print "The Pose is not yet"+str(robot_name)+" available... Please try again later"
            else:
                handle_turtle_pose(pose_now, robot_name)
        rate.sleep()


class GazeboModel(object):
    def __init__(self, robots_name_list = ['mobile_base_2', 'mobile_base_1']):
    
        # We wait for the topic to be available and when it is then retrive the index of each model
        # This was separated from callbal to avoid doing this in each callback
        self._robots_models_dict = {}
        self._robots_pose_list = []
        self._robots_index_dict = {}
        self._robots_name_list = robots_name_list
        
        self.rate_get_robot_index_object = rospy.Rate(1) # 1Hz
        
        self.get_robot_index()
        
        # We now start the suscriber once we have the indexes of each model
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
    
    def get_robot_index(self):
        
        
        data = None
        found_all_robot_names = False
        while not found_all_robot_names:
            rospy.loginfo("Retrieveing Model indexes ")
            try:
                data = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5)
                # Save it in the format {"robot1":4,"robot2":2}
                if data:
                    # Here we have model_states data, but not guarantee that the models we want are there
                    
                    for robot_name in self._robots_name_list:
                        robot_name_found = self.update_robot_index(data, robot_name)
                        if robot_name_found:
                            pass
                        else:
                            break
                    
                    found_all_robot_names = len(self._robots_index_dict) == len(self._robots_name_list)
                else:
                    rospy.loginfo("Topic /gazebo/model_states NOT Ready yet, trying again ")
            
            except Exception as e:
                s = str(e)
                rospy.loginfo("Error in get_robot_index = "+ s)
            
            self.rate_get_robot_index_object.sleep()
        
        assert found_all_robot_names, "NOT all the robot names were found"
        rospy.loginfo("Final robots_index_dict =  %s ", str(self._robots_index_dict))
    
    def update_robot_index(self,data, robot_name): #data type : ModelStates
        try:
            index = data.name.index(robot_name)
            self._robots_index_dict[robot_name] = index
            found = True
        except ValueError:
            rospy.loginfo("Robot Name="+str(robot_name)+", is NOT in model_state, trying again")
            found = False
        
        return found
    
    def callback(self,data):
        
        for robot_name in self._robots_name_list:
            # Retrieve the corresponding index
            robot_name_found = self.update_robot_index(data, robot_name) #data type : ModelStates
            if robot_name_found:
                data_index = self._robots_index_dict[robot_name]
                # Get the pose data from theat index
                try:
                    data_pose = data.pose[data_index]
                except IndexError:
                    rospy.logwarn("The model with data index "+str(data_index)+", something went wrong.")
                    data_pose = None
            else:
                data_pose = None
            # Save the pose inside the dict {"robot1":pose1,"robot2":pose2}
            self._robots_models_dict[robot_name] = data_pose

            
    def get_model_gazebo_pose(self,robot_name):
        
        pose_now = None
        
        try:
            pose_now = self._robots_models_dict[robot_name]
        except Exception as e:
            s = str(e)
            rospy.loginfo("Error, The _robots_models_dict is not ready = "+ s)

        return pose_now




if __name__ == '__main__':
    try:
        publisher_of_tf()
    except rospy.ROSInterruptException:
        pass