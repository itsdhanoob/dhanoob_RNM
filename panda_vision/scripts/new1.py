#!/usr/bin/env python

import numpy as np 
import open3d as o3d

import copy


################## reading image from the bag file #########################
def pcd_maker():
    pcd = o3d.io.read_point_cloud("/home/rnm/Desktop/11/1656951080775463.pcd" )    #(1656950884373885.pcd, 1.25dis)  (1656951080775463.pcd,)
    pcd.estimate_normals()
    pcd_ds = pcd.voxel_down_sample(voxel_size = 0.015)
    pcd_ds.remove_radius_outlier(nb_points=16, radius=0.05)
    o3d.visualization.draw_geometries_with_editing([pcd_ds])

    ################### reading the cropped image #################################

    pcd = o3d.io.read_point_cloud("/home/rnm/Desktop/11/cropped_0001.ply")     #("/home/rnm/catkin_ws/src/panda_vision/scripts/cropped_03.ply")
    pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    pcd.estimate_normals()
    o3d.visualization.draw_geometries([pcd])
    return pcd
  

################### generating a mesh on the skeleton ########################
def mesh():
    mesh = o3d.io.read_triangle_mesh("/home/rnm/Downloads/data/scanning/Skeleton_Target.stl")
    point_cld = mesh.sample_points_poisson_disk(10000)
    point_cld_scale = np.asarray(point_cld.points)
    point_cld_scale = point_cld_scale/1000
    pcd_skt = o3d.geometry.PointCloud()
    pcd_skt.points = o3d.utility.Vector3dVector(point_cld_scale)

    #o3d.visualization.draw_geometries([mesh])

    # o3d.visualization.draw_geometries([pcd_skt])
    return pcd_skt

################# tranformation between current scene and target ###############

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

################## pre proccesing for global registration ######################

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

#################### input data set for global registration #####################

def prepare_dataset(voxel_size):
    print(":: Loading source and target")
    source = mesh()
    target = pcd_maker()
    
    trans_init =  np.asarray([[0, -1,  0 , -0.00219754],
                            [-1, 0,  0,  0.00376681 ], # moved it cw 
                             [0, 0 ,-1,  0.14037132], #change in rotation
                             [0 ,0,   0,  1]])


    # trans_init= np.asarray([[-1.0, 0.0, 0.0, 0.0], 
    #                         [0.0, 1.0, 0.0, 0.0],
    #                          [0.0, 0.0, -1.0, 0.], 
    #                          [0.0, 0.0, 0.0, 1.0]])


    source.estimate_normals() 
    target.estimate_normals()
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

################### fast global registration #######################

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):

    distance_threshold = voxel_size * 1.25
    # print(":: Apply fast global registration with distance threshold %.3f" \
    #         % distance_threshold)
    # result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
    #     source_down, target_down, source_fpfh, target_fpfh,
    #     o3d.pipelines.registration.FastGlobalRegistrationOption(
    #         maximum_correspondence_distance=distance_threshold))
    # return result

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(80000000, 0.999))
    return result



####################### refining the fast global with point to plane icp #################################################### 

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)  
    result = o3d.pipelines.registration.registration_icp(source, target, distance_threshold, result_fast.transformation,
                                               o3d.pipelines.registration.TransformationEstimationPointToPlane())
    
    return result


if __name__ == "__main__":
    
    voxel_size = 0.09
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)

   
    
    result_fast = execute_fast_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)

    #print(result_fast)
    #print(result_fast.transformation)
    draw_registration_result(source_down, target_down, result_fast.transformation)


    result_icp = refine_registration(source, target, source_fpfh, target_fpfh, voxel_size)
    print(result_icp)
    draw_registration_result(source, target, result_icp.transformation)
    source.transform(result_icp.transformation)
    combined = source
    print(source)
    combined+= target
    print(target)

    print(result_icp.transformation)

    target = o3d.visualization.VisualizerWithEditing()
    target.create_window()
    target.add_geometry(source.transform(result_icp.transformation))
    target.run()
    target.destroy_window()
    tgt_pts= []
    tgt_pts= target.get_picked_points()  # shift+lmb to select target point 


    # np_combined = np.asarray(combined.points)
    np_src=np.asarray(source.points)
    start_point=np_src[tgt_pts[0]],(3,1)
    # start_point.estimate_normals()
    tgt_p=np.reshape(np_src[tgt_pts[1]],(3,1))
    # print(tgt_p)

    # print("Starting point is :",start_point)


    aa=np.r_[tgt_p,np.reshape([1],(1,1))]
    print(aa)

    alpha=np.dot(result_icp.transformation, aa)
    print(alpha)
    
    # comb = np_combined[tgt_pts[0],np.newaxis]
    # comb=np.append(comb, 1.0)
    # comb= np.transpose(comb)
    # print(comb)

    # trans= np.matmul(result_icp.transformation,comb)
    # print(trans)




    # np_combined = np.asarray(source.points)
    # comb = np_combined[tgt_pts[0],np.newaxis]
    # comb= np.transpose(comb)
    # print(comb)



    # print(type(tgt_pts[0]))
    
    # print(np.shape(result_icp.transformation))

    # print(np.shape(comb))
    # # print(type.source)

    # print(type(source.points))
    # print(np.shape(source.points))

    # print(type(source))

    # print(type(source.transform(result_fast.transformation)))

    # src_np= np.asarray(source.points)

    # print(src_np[0])

    # src_tf=(source.transform(result_icp.transformation))
    # th=np.asarray(src_tf.points)
    # print(th[0])


    # b=np.asarray(source.points)
    # comb= source.points[[tgt_pts[0]]]
    # print(comb)
    # np_combined = np.asarray(combined.points)

    # np_combined = np.asarray(source.points)
    # comb = np_combined[tgt_pts[0],]
    # print(comb)
    # A = np.zeros((4,1))
    # A[4][0] = 1
    # for i in range(3):
    #     A[i][0] = comb[i]

    # print(A)

