#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <plan_env/raycast.h>

/*
 * 文件：raycast.cpp
 * 作用：实现体素网格中沿射线遍历（raycast）的核心算法和辅助类RayCaster
 * 说明：基于Amanatides & Woo的快速体素遍历算法，提供两种Raycast接口和逐步遍历的RayCaster类
 * 注意：仅追加注释，保留原有英文注释与代码逻辑
 */

int signum(int x)
{
    return x == 0 ? 0 : x < 0 ? -1
                              : 1; // 返回x的符号：负->-1，零->0，正->1
}

double mod(double value, double modulus)
{
    return fmod(fmod(value, modulus) + modulus, modulus); // 对value取模，保证结果为[0, modulus)
}

double intbound(double s, double ds)
{
    // Find the smallest positive t such that s+t*ds is an integer.
    if (ds < 0)
    {
        return intbound(-s, -ds);
    }
    else
    {
        s = mod(s, 1);
        // problem is now s+t*ds = 1
        return (1 - s) / ds; // 计算从s沿方向ds到达下一个整数边界所需的参数t
    }
}

void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
             const Eigen::Vector3d &max, int &output_points_cnt, Eigen::Vector3d *output)
{
    //    std::cout << start << ' ' << end << std::endl;
    // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
    // by John Amanatides and Andrew Woo, 1987
    // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
    // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
    // Extensions to the described algorithm:
    //   • Imposed a distance limit.
    //   • The face passed through to reach the current cube is provided to
    //     the callback.

    // The foundation of this algorithm is a parameterized representation of
    // the provided ray,
    //                    origin + t * direction,
    // except that t is not actually stored; rather, at any given point in the
    // traversal, we keep track of the *greater* t values which we would have
    // if we took a step sufficient to cross a cube boundary along that axis
    // (i.e. change the integer part of the coordinate) in the variables
    // tMaxX, tMaxY, and tMaxZ.

    // Cube containing origin point.
    int x = (int)std::floor(start.x());        // 起点所在体素的整数坐标x
    int y = (int)std::floor(start.y());        // 起点所在体素的整数坐标y
    int z = (int)std::floor(start.z());        // 起点所在体素的整数坐标z
    int endX = (int)std::floor(end.x());       // 终点所在体素的整数坐标x
    int endY = (int)std::floor(end.y());       // 终点所在体素的整数坐标y
    int endZ = (int)std::floor(end.z());       // 终点所在体素的整数坐标z
    Eigen::Vector3d direction = (end - start); // 射线方向向量（终点-起点），未归一化
    double maxDist = direction.squaredNorm();  // 射线的平方长度，用于距离裁剪

    // Break out direction vector.
    double dx = endX - x; // 离散化后在体素坐标系中的x方向增量
    double dy = endY - y; // 离散化后在体素坐标系中的y方向增量
    double dz = endZ - z; // 离散化后在体素坐标系中的z方向增量

    // Direction to increment x,y,z when stepping.
    int stepX = (int)signum((int)dx); // 在x轴上步进方向（-1,0,1）
    int stepY = (int)signum((int)dy); // 在y轴上步进方向（-1,0,1）
    int stepZ = (int)signum((int)dz); // 在z轴上步进方向（-1,0,1）

    // See description above. The initial values depend on the fractional
    // part of the origin.
    double tMaxX = intbound(start.x(), dx); // 到x轴下一个整数边界的参数t
    double tMaxY = intbound(start.y(), dy); // 到y轴下一个整数边界的参数t
    double tMaxZ = intbound(start.z(), dz); // 到z轴下一个整数边界的参数t

    // The change in t when taking a step (always positive).
    double tDeltaX = ((double)stepX) / dx; // 沿x轴每步t的增量
    double tDeltaY = ((double)stepY) / dy; // 沿y轴每步t的增量
    double tDeltaZ = ((double)stepZ) / dz; // 沿z轴每步t的增量

    // Avoids an infinite loop.
    if (stepX == 0 && stepY == 0 && stepZ == 0)
        return;

    double dist = 0;
    while (true)
    {
        if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z())
        {
            output[output_points_cnt](0) = x; // 记录当前体素x
            output[output_points_cnt](1) = y; // 记录当前体素y
            output[output_points_cnt](2) = z; // 记录当前体素z

            output_points_cnt++; // 输出计数自增
            dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                        (z - start(2)) * (z - start(2))); // 计算当前体素中心到起点的欧氏距离

            if (dist > maxDist)
                return; // 若超过射线最大长度则终止

            /*            if (output_points_cnt > 1500) {
                            std::cerr << "Error, too many racyast voxels." <<
               std::endl;
                            throw std::out_of_range("Too many raycast voxels");
                        }*/
        }

        if (x == endX && y == endY && z == endZ)
            break;

        // tMaxX stores the t-value at which we cross a cube boundary along the
        // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
        // chooses the closest cube boundary. Only the first case of the four
        // has been commented in detail.
        if (tMaxX < tMaxY)
        {
            if (tMaxX < tMaxZ)
            {
                // Update which cube we are now in.
                x += stepX; // 沿x方向移动到下一个体素
                // Adjust tMaxX to the next X-oriented boundary crossing.
                tMaxX += tDeltaX; // 更新x方向的边界参数
            }
            else
            {
                z += stepZ;       // 沿z方向移动到下一个体素
                tMaxZ += tDeltaZ; // 更新z方向的边界参数
            }
        }
        else
        {
            if (tMaxY < tMaxZ)
            {
                y += stepY;       // 沿y方向移动到下一个体素
                tMaxY += tDeltaY; // 更新y方向的边界参数
            }
            else
            {
                z += stepZ;       // 沿z方向移动到下一个体素
                tMaxZ += tDeltaZ; // 更新z方向的边界参数
            }
        }
    }
}

void Raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &min,
             const Eigen::Vector3d &max, std::vector<Eigen::Vector3d> *output)
{
    //    std::cout << start << ' ' << end << std::endl;
    // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
    // by John Amanatides and Andrew Woo, 1987
    // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
    // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
    // Extensions to the described algorithm:
    //   • Imposed a distance limit.
    //   • The face passed through to reach the current cube is provided to
    //     the callback.

    // The foundation of this algorithm is a parameterized representation of
    // the provided ray,
    //                    origin + t * direction,
    // except that t is not actually stored; rather, at any given point in the
    // traversal, we keep track of the *greater* t values which we would have
    // if we took a step sufficient to cross a cube boundary along that axis
    // (i.e. change the integer part of the coordinate) in the variables
    // tMaxX, tMaxY, and tMaxZ.

    // Cube containing origin point.
    int x = (int)std::floor(start.x());        // 起点所在体素的整数坐标x
    int y = (int)std::floor(start.y());        // 起点所在体素的整数坐标y
    int z = (int)std::floor(start.z());        // 起点所在体素的整数坐标z
    int endX = (int)std::floor(end.x());       // 终点所在体素的整数坐标x
    int endY = (int)std::floor(end.y());       // 终点所在体素的整数坐标y
    int endZ = (int)std::floor(end.z());       // 终点所在体素的整数坐标z
    Eigen::Vector3d direction = (end - start); // 射线方向向量（终点-起点），未归一化
    double maxDist = direction.squaredNorm();  // 射线的平方长度，用于距离裁剪

    // Break out direction vector.
    double dx = endX - x; // 离散化后在体素坐标系中的x方向增量
    double dy = endY - y; // 离散化后在体素坐标系中的y方向增量
    double dz = endZ - z; // 离散化后在体素坐标系中的z方向增量

    // Direction to increment x,y,z when stepping.
    int stepX = (int)signum((int)dx); // 在x轴上步进方向（-1,0,1）
    int stepY = (int)signum((int)dy); // 在y轴上步进方向（-1,0,1）
    int stepZ = (int)signum((int)dz); // 在z轴上步进方向（-1,0,1）

    // See description above. The initial values depend on the fractional
    // part of the origin.
    double tMaxX = intbound(start.x(), dx); // 到x轴下一个整数边界的参数t
    double tMaxY = intbound(start.y(), dy); // 到y轴下一个整数边界的参数t
    double tMaxZ = intbound(start.z(), dz); // 到z轴下一个整数边界的参数t

    // The change in t when taking a step (always positive).
    double tDeltaX = ((double)stepX) / dx; // 沿x轴每步t的增量
    double tDeltaY = ((double)stepY) / dy; // 沿y轴每步t的增量
    double tDeltaZ = ((double)stepZ) / dz; // 沿z轴每步t的增量

    output->clear();

    // Avoids an infinite loop.
    if (stepX == 0 && stepY == 0 && stepZ == 0)
        return;

    double dist = 0;
    while (true)
    {
        if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z())
        {
            output->push_back(Eigen::Vector3d(x, y, z)); // 将当前体素坐标加入输出列表

            dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm(); // 计算当前体素中心到起点的平方距离

            if (dist > maxDist)
                return; // 若超过射线最大长度则终止

            if (output->size() > 1500)
            {
                std::cerr << "Error, too many racyast voxels." << std::endl;
                throw std::out_of_range("Too many raycast voxels");
            }
        }

        if (x == endX && y == endY && z == endZ)
            break;

        // tMaxX stores the t-value at which we cross a cube boundary along the
        // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
        // chooses the closest cube boundary. Only the first case of the four
        // has been commented in detail.
        if (tMaxX < tMaxY)
        {
            if (tMaxX < tMaxZ)
            {
                // Update which cube we are now in.
                x += stepX;
                // Adjust tMaxX to the next X-oriented boundary crossing.
                tMaxX += tDeltaX;
            }
            else
            {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
        else
        {
            if (tMaxY < tMaxZ)
            {
                y += stepY;
                tMaxY += tDeltaY;
            }
            else
            {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
    }
}

bool RayCaster::setInput(const Eigen::Vector3d &start,
                         const Eigen::Vector3d &end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */
)
{
    start_ = start;
    end_ = end;
    // max_ = max;
    // min_ = min;

    x_ = (int)std::floor(start_.x());    // 起点所在体素的整数坐标x
    y_ = (int)std::floor(start_.y());    // 起点所在体素的整数坐标y
    z_ = (int)std::floor(start_.z());    // 起点所在体素的整数坐标z
    endX_ = (int)std::floor(end_.x());   // 终点所在体素的整数坐标x
    endY_ = (int)std::floor(end_.y());   // 终点所在体素的整数坐标y
    endZ_ = (int)std::floor(end_.z());   // 终点所在体素的整数坐标z
    direction_ = (end_ - start_);        // 射线方向向量（终点-起点）
    maxDist_ = direction_.squaredNorm(); // 射线的平方长度

    // Break out direction vector.
    dx_ = endX_ - x_; // 离散化后在体素坐标系中的x方向增量
    dy_ = endY_ - y_; // 离散化后在体素坐标系中的y方向增量
    dz_ = endZ_ - z_; // 离散化后在体素坐标系中的z方向增量

    // Direction to increment x,y,z when stepping.
    stepX_ = (int)signum((int)dx_); // 在x轴上步进方向（-1,0,1）
    stepY_ = (int)signum((int)dy_); // 在y轴上步进方向（-1,0,1）
    stepZ_ = (int)signum((int)dz_); // 在z轴上步进方向（-1,0,1）

    // See description above. The initial values depend on the fractional
    // part of the origin.
    tMaxX_ = intbound(start_.x(), dx_); // 到x轴下一个整数边界的参数t
    tMaxY_ = intbound(start_.y(), dy_); // 到y轴下一个整数边界的参数t
    tMaxZ_ = intbound(start_.z(), dz_); // 到z轴下一个整数边界的参数t

    // The change in t when taking a step (always positive).
    tDeltaX_ = ((double)stepX_) / dx_; // 沿x轴每步t的增量
    tDeltaY_ = ((double)stepY_) / dy_; // 沿y轴每步t的增量
    tDeltaZ_ = ((double)stepZ_) / dz_; // 沿z轴每步t的增量

    dist_ = 0;

    step_num_ = 0;

    // Avoids an infinite loop.
    if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
        return false;
    else
        return true;
}

bool RayCaster::step(Eigen::Vector3d &ray_pt)
{
    // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
    // z_ >= min_.z() && z_ <
    // max_.z())
    ray_pt = Eigen::Vector3d(x_, y_, z_); // 当前体素坐标

    // step_num_++;

    // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

    if (x_ == endX_ && y_ == endY_ && z_ == endZ_)
    {
        return false; // 已到达终点体素，遍历结束
    }

    // if (dist_ > maxDist_)
    // {
    //   return false;
    // }

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX_ < tMaxY_)
    {
        if (tMaxX_ < tMaxZ_)
        {
            // Update which cube we are now in.
            x_ += stepX_; // 沿x方向前进一步
            // Adjust tMaxX to the next X-oriented boundary crossing.
            tMaxX_ += tDeltaX_; // 更新x方向边界参数
        }
        else
        {
            z_ += stepZ_;       // 沿z方向前进一步
            tMaxZ_ += tDeltaZ_; // 更新z方向边界参数
        }
    }
    else
    {
        if (tMaxY_ < tMaxZ_)
        {
            y_ += stepY_;       // 沿y方向前进一步
            tMaxY_ += tDeltaY_; // 更新y方向边界参数
        }
        else
        {
            z_ += stepZ_;       // 沿z方向前进一步
            tMaxZ_ += tDeltaZ_; // 更新z方向边界参数
        }
    }

    return true;
}