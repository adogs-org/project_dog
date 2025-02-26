//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#include "ocs2_quadruped_controller/wbc/HoQp.h"

#include <qpOASES.hpp>//引入qp求解器
#include <utility>//标准数学函数

namespace ocs2::legged_robot {
    HoQp::HoQp(Task task, HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
        initVars();//初始化
        formulateProblem();//构建初始化问题
        solveProblem();//求解结果
        // For next problem
        buildZMatrix();//构建零空间，实现任务分层优先级 优先级不同效果也不同
        stackSlackSolutions();//使用栈 储存结果
    }

    void HoQp::initVars() {
        // Task variables
        numSlackVars_ = task_.d_.rows();//当前任务的松弛变量
        hasEqConstraints_ = task_.a_.rows() > 0;//是否有等式约束
        hasIneqConstraints_ = numSlackVars_ > 0;//是否有不等式约束

        // Pre-Task variables
        if (higherProblem_ != nullptr) {//上层问题指的是需要求解出的高优先级任务
            stackedZPrev_ = higherProblem_->getStackedZMatrix();//获取上层的零空间矩阵
            stackedTasksPrev_ = higherProblem_->getStackedTasks();//获取上层任务
            stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();//上层松弛变量解
            xPrev_ = higherProblem_->getSolutions();//求解出上层的优化解
            numPrevSlackVars_ = higherProblem_->getSlackedNumVars();//储存上层松弛变量的个数

            numDecisionVars_ = stackedZPrev_.cols();//决策变量个数
        } else {
            numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols());//确定当前任务的决策变量个数

            stackedTasksPrev_ = Task(numDecisionVars_);//
            stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_);//设置为单位矩阵
            stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);//设置为空矩阵
            xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);//没有上层优化解
            numPrevSlackVars_ = 0;//没有上层松弛变量
        }

        stackedTasks_ = task_ + stackedTasksPrev_;//将当前任务与上层任务合并

        // Init convenience matrices
        eyeNvNv_ = matrix_t::Identity(numSlackVars_, numSlackVars_);//单位矩阵用于松弛变量
        zeroNvNx_ = matrix_t::Zero(numSlackVars_, numDecisionVars_);//零矩阵，用于矩阵拼接
    }

    void HoQp::formulateProblem() {
        buildHMatrix();//构建二次项矩阵
        buildCVector();//构建目标向量c 表示最大最小
        buildDMatrix();//构建不等式约束矩阵
        buildFVector();//构建不等式约束向量f
    }

    void HoQp::buildHMatrix() {
        matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);// 初始化用于计算零空间的中间变量

        if (hasEqConstraints_) {//如果存在等式约束
            // Make sure that all eigenvalues of A_t_A are non-negative, which could arise due to numerical issues
            matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;//计算当前任务的等式约束与零空间矩阵的乘积
            zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity( // 计算 (A * Z)^T * (A * Z)
                         numDecisionVars_, numDecisionVars_); // 添加小偏移以避免数值问题
            // This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
        } else {
            zTaTaz.setZero(); // 如果没有等式约束，则设置为零
        }
           // 构建最终的 H 矩阵，包括等式部分和松弛变量部分
        h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_) // clang-format off
              << zTaTaz, zeroNvNx_.transpose(),
              zeroNvNx_, eyeNvNv_) // clang-format on
                .finished();
    }

    void HoQp::buildCVector() {
        vector_t c = vector_t::Zero(numDecisionVars_ + numSlackVars_);
        vector_t zeroVec = vector_t::Zero(numSlackVars_);

        vector_t temp(numDecisionVars_);
        if (hasEqConstraints_) {
            temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
        } else {
            temp.setZero();
        }

        c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
    }

    void HoQp::buildDMatrix() {
        matrix_t stackedZero = matrix_t::Zero(numPrevSlackVars_, numSlackVars_);

        matrix_t dCurrZ;
        if (hasIneqConstraints_) {
            dCurrZ = task_.d_ * stackedZPrev_;
        } else {
            dCurrZ = matrix_t::Zero(0, numDecisionVars_);
        }

        // NOTE: This is upside down compared to the paper,
        // but more consistent with the rest of the algorithm
        d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_) // clang-format off
              << zeroNvNx_, -eyeNvNv_,
              stackedTasksPrev_.d_ * stackedZPrev_, stackedZero,
              dCurrZ, -eyeNvNv_) // clang-format on
                .finished();
    }

    void HoQp::buildFVector() {
        vector_t zeroVec = vector_t::Zero(numSlackVars_);

        vector_t fMinusDXPrev;
        if (hasIneqConstraints_) {
            fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;
        } else {
            fMinusDXPrev = vector_t::Zero(0);
        }

        f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
              stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
                .finished();
    }

    void HoQp::buildZMatrix() {
        if (hasEqConstraints_) {
            assert((task_.a_.cols() > 0));
            stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
        } else {
            stackedZ_ = stackedZPrev_;
        }
    }

    void HoQp::solveProblem() {
        auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
        qpOASES::Options options;
        options.setToMPC();//配置选项 选择mpc
        options.printLevel = qpOASES::PL_LOW;//打印级别
        qpProblem.setOptions(options);
        int nWsr = 20;//最大迭代次数

        qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);//初始化
        vector_t qpSol(numDecisionVars_ + numSlackVars_);//储存优化解

        qpProblem.getPrimalSolution(qpSol.data());//获取解

        decisionVarsSolutions_ = qpSol.head(numDecisionVars_);//提取决策变量的解
        slackVarsSolutions_ = qpSol.tail(numSlackVars_); // 提取松弛变量的解
    }

// 累积松弛变量的解，用于下一层次优化
    void HoQp::stackSlackSolutions() {
        if (higherProblem_ != nullptr) {
            stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(),
                                                         slackVarsSolutions_);//如果有上层问题，合并上层的松弛解
        } else {
            stackedSlackVars_ = slackVarsSolutions_;
        }
    }
} // namespace legged
