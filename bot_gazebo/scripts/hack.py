import rospy, time
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler
import random, rospkg
print '******************************************'
rospack = rospkg.RosPack()
path = rospack.get_path('ebot_gazebo')
print path
sdf_model1 = open(path + '/models/coke_can/model.sdf', 'r').read()
sdf_model2 = open(path + '/models/glue/model.sdf', 'r').read()
sdf_model3 = open(path + '/models/soap2/model.sdf', 'r').read()
sdf_model4 = open(path + '/models/robot_wheels/model.sdf', 'r').read()
sdf_model5 = open(path + '/models/adhesive/model.sdf', 'r').read()
sdf_model6 = open(path + '/models/soap/model.sdf', 'r').read()
sdf_model7 = open(path + '/models/eYIFI/model.sdf', 'r').read()
sdf_model8 = open(path + '/models/water_glass/model.sdf', 'r').read()

def create_model_request(sdf_model, modelname, px, py, pz, rr, rp, ry):
    model = deepcopy(sdf_model)
    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = model
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz
    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]
    return req


def random_model_pose(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y):
    x_position = [
     p1_x, p2_x, p3_x]
    y_position = [p1_y, p2_y, p3_y]
    random.shuffle(x_position)
    random.shuffle(y_position)
    i = round(random.uniform(0, 3.14), 2)
    pose1 = [
     x_position[0], y_position[0], 0.87, 0.0, 0.0, 0.0]
    pose2 = [x_position[1], y_position[1], 0.87, 0.0, 0.0, 0.0]
    pose3 = [x_position[2], y_position[2], 0.87, 0.0, 0.0, 1.053]
    return (
     pose1, pose2, pose3)


def pantry_items_pose():
    p_x_position = [
     15.23, 15.48, 10.71, 10.72, 10.82, 10.72, 15.15, 15.32]
    p_y_position = [-0.86, -1.08, -0.956, -1.23, -1.045, -1.24, -1.143, -1.157]
    random.shuffle(p_x_position)
    random.shuffle(p_y_position)
    p_pose1 = [15.22, -0.748, 0.87, 0.0, 0.0, 0.0]
    p_pose2 = [p_x_position[3], p_y_position[3], 0.87, 0.0, 0.0, 0.0]
    return (
     p_pose1, p_pose2)


def inventory_items_pose():
    i_position = [
     [
      26.7, -2.984], [26.78, -3.21], [26.395, -3.45], [26.29, -3.613], [26.33, -3.88]]
    random.shuffle(i_position)
    i_pose1 = [i_position[0][0], i_position[0][1], 1.17, 0.0, 0.0, 0.0]
    i_pose2 = [i_position[1][0], i_position[1][1], 1.17, 0.0, 0.0, 0.0]
    i_pose3 = [26.55, -3.23, 1.17, 0.0, 0.0, 0.0]
    i_pose4 = [i_position[3][0], i_position[3][1], 1.17, 0.0, 0.0, 0.0]
    i_pose5 = [i_position[4][0], i_position[4][1], 1.17, 0.0, 0.0, 0.0]
    i_pose6 = [i_position[2][0], i_position[2][1], 1.17, 0.0, 0.0, 0.0]
    return (i_pose1, i_pose2, i_pose3, i_pose4, i_pose5, i_pose6)


if __name__ == '__main__':
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo('Waiting for /gazebo/spawn_sdf_model service...')
    spawn_srv.wait_for_service()
    rospy.loginfo('Connected to service!')
    pantry_pose1, pantry_pose2 = pantry_items_pose()
    rospy.loginfo('Spawning coke_can')
    p_req1 = create_model_request(sdf_model1, 'coke_can', pantry_pose1[0], pantry_pose1[1], pantry_pose1[2], pantry_pose1[3], pantry_pose1[4], pantry_pose1[5])
    spawn_srv.call(p_req1)
    rospy.loginfo('Spawning water_glass')
    p_req2 = create_model_request(sdf_model8, 'water_glass', pantry_pose2[0], pantry_pose2[1], pantry_pose2[2], pantry_pose2[3], pantry_pose2[4], pantry_pose2[5])
    spawn_srv.call(p_req2)
    in_pose1, in_pose2, in_pose3, in_pose4, in_pose5, in_pose6 = inventory_items_pose()
    rospy.loginfo('Spawning adhesive')
    i_req1 = create_model_request(sdf_model5, 'adhesive', in_pose1[0], in_pose1[1], in_pose1[2], in_pose1[3], in_pose1[4], in_pose1[5])
    spawn_srv.call(i_req1)
    rospy.loginfo('Spawning glue')
    i_req2 = create_model_request(sdf_model2, 'glue', in_pose2[0], in_pose2[1], in_pose2[2], in_pose2[3], in_pose2[4], in_pose2[5])
    spawn_srv.call(i_req2)
    rospy.loginfo('Spawning battery')
    i_req3 = create_model_request(sdf_model3, 'battery', in_pose3[0], in_pose3[1], in_pose3[2], in_pose3[3], in_pose3[4], in_pose3[5])
    spawn_srv.call(i_req3)
    rospy.loginfo('Spawning FPGA board')
    i_req4 = create_model_request(sdf_model6, 'FPGA_board', in_pose4[0], in_pose4[1], in_pose4[2], in_pose4[3], in_pose4[4], in_pose4[5])
    spawn_srv.call(i_req4)
    rospy.loginfo('Spawning Pair of wheels package')
    i_req5 = create_model_request(sdf_model4, 'robot_wheels', in_pose5[0], in_pose5[1], in_pose5[2], in_pose5[3], in_pose5[4], in_pose5[5])
    spawn_srv.call(i_req5)
    rospy.loginfo('Spawning eYFi board')
    i_req6 = create_model_request(sdf_model7, 'eYFi_board', in_pose6[0], in_pose6[1], in_pose6[2], in_pose6[3], in_pose6[4], in_pose6[5])
    spawn_srv.call(i_req6)
    pose1, pose2, pose3 = random_model_pose(8.108564, 3.239278, 7.971279, 3.393995, 7.84, 3.24)
    rospy.loginfo('Spawning glue')
    req2 = create_model_request(sdf_model2, 'glue2', pose3[0], pose3[1], pose3[2], pose3[3], pose3[4], pose3[5])
    spawn_srv.call(req2)
    rospy.loginfo('Spawning adhesive')
    req3 = create_model_request(sdf_model5, 'adhesive2', pose1[0], pose1[1], pose1[2], pose1[3], pose1[4], pose1[5])
    spawn_srv.call(req3)
