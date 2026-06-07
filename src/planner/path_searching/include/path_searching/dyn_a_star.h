#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    typedef enum enum_state
    {
        OPENSET = 1,   // 当前节点在开放列表中 开放列表一般采用自动排序的数据结构例如multimap 里面存放的节点按照排序等待扩展
        CLOSEDSET = 2, // 当前节点在关闭列表中 已经被访问过，不需要再次加入开放列表
        UNDEFINED = 3  // 当前节点还没有被访问
    }enum_state;

    int rounds{0}; // Distinguish every call
	enum_state state{UNDEFINED}; // 列表初始化语法
	Eigen::Vector3i index; // 节点索引

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

// priority_queue 内部逻辑是：如果 comp(a, b)为 true，说明 a 的优先级比 b 低。所以会把 b 放在 a 的前面。 
class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2); // 对角线启发式
	double getManhHeu(GridNodePtr node1, GridNodePtr node2); // 曼哈顿启发式
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2); // 欧氏启发式
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);// 采用对角线启发式

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_;

    // 这里声明了一个 最小堆（openSet），里面存放 GridNodePtr。std::priority_queue 默认是 大顶堆（最大值优先）。如果你要让 fScore 小的节点排在前面，就需要提供一个 比较器。
    // priority_queue 的第三个模板参数 必须是一个“可调用对象”（函数对象/函数指针/lambda），它的签名要和 bool operator()(T a, T b) 类似。
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

    // A*搜索函数
	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
