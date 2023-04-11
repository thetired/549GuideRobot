/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kNumSubmapScanMatchersMetric = metrics::Gauge::Null();

// 返回submap的原点在local坐标系下的二维坐标
transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

/**
 * @brief 构造函数
 * 
 * @param[in] options 约束构造器的配置参数
 * @param[in] thread_pool map_builder中构造的线程池
 */
ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(absl::make_unique<common::Task>()),  //任务初始化，为空
      when_done_task_(absl::make_unique<common::Task>()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}  //是pose  graph lua里边的参数配置，认为地图更重要  usenomonotonic参数 false容易到局部最小值

ConstraintBuilder2D::~ConstraintBuilder2D() {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

/**
 * @brief 进行局部搜索窗口的约束计算(对局部子图进行回环检测)
 * 
 * @param[in] submap_id submap的id
 * @param[in] submap 单个submap
 * @param[in] node_id 节点的id
 * @param[in] constant_data 节点的数据
 * @param[in] initial_relative_pose 约束的初值
 */
// 匹配结点与子图再局部窗口约束计算的，不是匹配全部子图，而是子图一部分
void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  // 超过范围的不进行约束的计算
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) { // param: max_constraint_distance
    return;
  }
  // 根据参数配置添加约束的频率 采样器减少计算量 56个结点走得慢， 和子图匹配贡献很小，蛇者了采样率，不是每一个结点都匹配
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }

  absl::MutexLock locker(&mutex_);
  // 当when_done_正在处理任务时调用本函数, 报个警告
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }

  // 在队列中新建一个指向Constraint数据的指针 ，约束是一个双端队列，添加新约束，为空
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();  //获取约束指针
  
  // 为子图新建一个匹配器
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());

  // 生成个计算约束的任务
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                      constant_data, initial_relative_pose, *scan_matcher,
                      constraint);
  });

  // 等匹配器之后初始化才能进行约束的计算
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  // 将计算约束这个任务放入线程池等待执行 ，任务比较耗时，放入线程池种计算
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  // 将计算约束这个任务 添加到 finish_node_task_的依赖项中
  finish_node_task_->AddDependency(constraint_task_handle);
}

/**
 * @brief 进行全局搜索窗口的约束计算(对整体子图进行回环检测)
 * 
 * @param[in] submap_id submap的id
 * @param[in] submap 单个submap
 * @param[in] node_id 节点的id
 * @param[in] constant_data 节点的数据
 */
void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  
  // note: 对整体子图进行回环检测时没有距离的限制

  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  // 为子图新建一个匹配器
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  // 生成个计算全局约束的任务
  //设置新的任务，任务为空 ，任务为一个lambda表达式
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                      constant_data, transform::Rigid2d::Identity(),
                      *scan_matcher, constraint);
  });
  //任务添加了一个依赖项， 依赖scan_matcher ，等地图建完才能用，要用多分辨率地图
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  //任务放入到线程池里边
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  //finish_node_tast_为空
  finish_node_task_->AddDependency(constraint_task_handle);
}

// 告诉ConstraintBuilder2D的对象, 刚刚完成了一个节点的约束的计算
void ConstraintBuilder2D::NotifyEndOfNode() {
  absl::MutexLock locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  
  // 生成个任务: 将num_finished_nodes_自加, 记录完成约束计算节点的总个数
  //设置了任务， 为了标记一下结点约束全都执行完
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&mutex_);
    ++num_finished_nodes_;
  });

  // 将这个任务传入线程池中等待执行, 由于之前添加了依赖, 所以finish_node_task_一定会比计算约束更晚完成
  //依赖项设置完了 ，任务移动到这里
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));

  // move之后finish_node_task_就没有指向的地址了, 所以这里要重新初始化
  finish_node_task_ = absl::make_unique<common::Task>();
  // 设置when_done_task_依赖finish_node_task_handle
  when_done_task_->AddDependency(finish_node_task_handle); //when done  依赖这个 ，when done是空的
  ++num_started_nodes_;
}

// 约束计算完成之后执行一下回调函数
void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  CHECK(when_done_ == nullptr);

  // TODO(gaschler): Consider using just std::function, it can also be empty.
  // 将回调函数赋值给when_done_
  when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);

  // 生成 执行when_done_的任务
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  // 将任务放入线程池中等待执行
  thread_pool_->Schedule(std::move(when_done_task_));

  // when_done_task_的重新初始化
  when_done_task_ = absl::make_unique<common::Task>();
}

// 为每个子图新建一个匹配器
//构造ScanMatcher，这是为子图进行构造，一个子图建一个匹配器
const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  CHECK(grid);
  // 如果匹配器里已经存在, 则直接返回对应id的匹配器
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  //子图匹配器没有新建
  // submap_scan_matchers_新增加一个 key
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
  // 保存栅格地图的指针
  submap_scan_matcher.grid = grid;

  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options(); //获取参数配置 分枝定界的配置
  auto scan_matcher_task = absl::make_unique<common::Task>();
  // 生成一个将初始化匹配器的任务, 初始化时会计算多分辨率地图, 比较耗时
  //为什么将初始化的任务放入到线程池里边呢？ 由于 :FastCorrelativeScanMatcher2D 初始化生成地图的多分辨率地图，分辨率不一样 计算量比较大，费时放线程池里边
  scan_matcher_task->SetWorkItem(  //添加任务，添加的是一个lambda表达式
      [&submap_scan_matcher, &scan_matcher_options]() {
        // 进行匹配器的初始化, 与多分辨率地图的创建
        submap_scan_matcher.fast_correlative_scan_matcher =
            absl::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(   //初始化 拷贝栅格栅格地图
                *submap_scan_matcher.grid, scan_matcher_options);
      });  //任务没有执行
  // 将初始化匹配器的任务放入线程池中, 并且将任务的智能指针保存起来
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));  //把任务传入到线程池ideSchedule里边 ，这个任务没有依赖项， 很快执行

  return &submap_scan_matchers_.at(submap_id);
}

/**
 * @brief 计算节点和子图之间的一个约束(回环检测)
 *        用基于分支定界算法的匹配器进行粗匹配,然后用ceres进行精匹配
 * 
 * @param[in] submap_id submap的id
 * @param[in] submap 地图数据
 * @param[in] node_id 节点id
 * @param[in] match_full_submap 是局部匹配还是全子图匹配
 * @param[in] constant_data 节点数据  ，点云信息
 * @param[in] initial_relative_pose 约束的初值
 * @param[in] submap_scan_matcher 匹配器
 * @param[out] constraint 计算出的约束
 */
//只传入一个结点id，函数需要循环的，每次传只穿一个结点和子图，计算除一条约束
//local  和global不一样的 , global 不需要初值，传入的是000  ，搜一下initial_realtive_pose从哪儿计算的
void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);

  // Step:1 得到节点在local frame下的坐标
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)

  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  // param: global_localization_min_score 对整体子图进行回环检测时的最低分数阈值，重定位或者回环参数不好，可以把参数改小
  // param: min_score 对局部子图进行回环检测时的最低分数阈值

  // Step:2 使用基于分支定界算法的匹配器进行粗匹配  //没仔细看
  //如果匹配全子图
  if (match_full_submap) {  //纯定位才进这里边，
    // 节点与全地图进行匹配
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(  //基于分枝定界的粗匹配 ，循环返回一个得分，成功得分和pose就会赋值
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
    } else {
      // 计算失败了就退出
      return;
    }
  } 
  else {
    // 节点与局部地图进行匹配
    kConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(  //返回计算成功与失败
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score(), &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
    } else {
      return;
    }
  }
  
  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.

  // Step:3 使用ceres进行精匹配, 就是前端扫描匹配使用的函数  ，和前端那个一样
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  // Step:4 获取节点到submap坐标系原点间的坐标变换  ，计算校准之后的约束
  // pose_estimate 是 节点在 loacl frame 下的坐标
  //最终需要的约束，子图原点指向tracking的约束坐标变换  ，子图内约束也是一样的
  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;

  // Step:5 返回计算后的约束
  //把约束放入到constraint里边
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(), //值很好不用改
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});

  // log相关 ，log_matches打印约束的日志
  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2) // c++11: std::setprecision(2) 保留2个小数点
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

// 将临时保存的所有约束数据传入回调函数, 并执行回调函数
void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);

    // 将计算完的约束进行保存
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }

    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    }

    // 这些约束已经保存过了, 就可以删掉了
    constraints_.clear();

    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  // 执行回调函数 HandleWorkQueue
  (*callback)(result);
}

// 获取完成约束计算节点的总个数
int ConstraintBuilder2D::GetNumFinishedNodes() {
  absl::MutexLock locker(&mutex_);
  return num_finished_nodes_;
}

// 删除指定submap_id的匹配器
void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
  auto* num_matchers = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_num_submap_scan_matchers",
      "Current number of constructed submap scan matchers");
  kNumSubmapScanMatchersMetric = num_matchers->Add({});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
