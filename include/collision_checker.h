//
// Created by airlab on 11/22/23.
//

#ifndef WINDOWBOPLANNER_COLLISION_CHECKER_H
#define WINDOWBOPLANNER_COLLISION_CHECKER_H
#include <iostream>
#include "boss_pch.h"


using namespace  fcl;

class CollisionChecker {
public:
    explicit CollisionChecker(const std::vector<std::vector<float>>& obstacles, const std::vector<std::vector<float>>& trajectory, float robot_radius=0.345, float obstacle_length=0.25):
    obstacles_(obstacles), trajectory_(trajectory), robot_radius_(robot_radius), obstacle_length_(obstacle_length)
    {

    }

    explicit CollisionChecker(const std::vector<std::array<double, 2>>& obstacles, const std::vector<std::array<double, 5>>& trajectory, float robot_radius=0.345, float obstacle_length=0.25):
    robot_radius_(robot_radius), obstacle_length_(obstacle_length)
    {
        for(auto& o:obstacles)
        {
            obstacles_.emplace_back(std::vector<float>{float(o[0]), float(o[1])});
        }

          for(auto& t:trajectory)
        {
            trajectory_.emplace_back(std::vector<float>{float(t[0]), float(t[1]), float(t[2])});
        }
    }

    std::vector<fcl::CollisionObjectf*> getCollisionObjectList(const std::vector<std::vector<float>>& obstacles, bool isRobot=true) const
    {
        std::vector<fcl::CollisionObjectf*> obs_list;
        for (auto o : obstacles){
            // R and T are the rotation matrix and translation vector
            fcl::Matrix3f R;
            fcl::Vector3f T;
            // code for setting R and T
            R = Eigen::Quaternionf(1, 0, 0, 0).matrix();
            T = Eigen::Vector3f(o[0], o[1], 0.500000);

            // transform is configured according to R and T
            fcl::Transform3f pose = fcl::Transform3f::Identity();
            pose.linear() = R;
            pose.translation() = T;
            if(isRobot)
            {
                auto geom = std::make_shared<fcl::Sphere<float>>(robot_radius_);
                fcl::CollisionObjectf* o1 = new fcl::CollisionObject<float>(
                        geom, pose);
                obs_list.push_back(o1);
            }
            else
            {
                auto geom = std::make_shared<fcl::Box<float>>(obstacle_length_, obstacle_length_, obstacle_length_);
                fcl::CollisionObjectf* o1 = new fcl::CollisionObject<float>(
                        geom, pose);
                obs_list.push_back(o1);
            }


        }
        return obs_list;
    }


    float get_min_dist() const
    {
        std::vector<fcl::CollisionObjectf*> obs_list = getCollisionObjectList(obstacles_, false);
        std::vector<fcl::CollisionObjectf*> robot_traj_list = getCollisionObjectList(trajectory_);
        BroadPhaseCollisionManagerf* manager1 = new DynamicAABBTreeCollisionManagerf();
        // Initialize the collision manager for the second group of objects.
        BroadPhaseCollisionManagerf* manager2 = new DynamicAABBTreeCollisionManagerf();
        manager1->registerObjects(obs_list);
        manager2->registerObjects(robot_traj_list);

        fcl::DefaultDistanceData<float> distance_data;
        manager1->setup();
        manager2->setup();

        manager2->distance(manager1, &distance_data, DefaultDistanceFunction);
        double min_distance = distance_data.result.min_distance;
        return min_distance;
    }

    bool collide() const
    {
        std::vector<fcl::CollisionObjectf*> obs_list = getCollisionObjectList(obstacles_, false);
        std::vector<fcl::CollisionObjectf*> robot_traj_list = getCollisionObjectList(trajectory_);
        BroadPhaseCollisionManagerf* manager1 = new DynamicAABBTreeCollisionManagerf();
        // Initialize the collision manager for the second group of objects.
        BroadPhaseCollisionManagerf* manager2 = new DynamicAABBTreeCollisionManagerf();
        manager1->registerObjects(obs_list);
        manager2->registerObjects(robot_traj_list);
        fcl::DefaultCollisionData<float> collision_data;
        manager1->setup();
        manager2->setup();

        manager2->collide(manager1, &collision_data, DefaultCollisionFunction);
        return  collision_data.result.isCollision();
    }


    ~CollisionChecker() {

    }

private:
    float robot_radius_, obstacle_length_;
    std::vector<std::vector<float>> obstacles_;
    std::vector<std::vector<float>> trajectory_;

};
#endif //WINDOWBOPLANNER_COLLISION_CHECKER_H
