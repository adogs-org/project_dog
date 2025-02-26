//
// Created by qiayuan on 2022/7/1.
//

#include "ocs2_quadruped_controller/wbc/WbcBase.h"

#include <utility>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace ocs2::legged_robot {
    WbcBase::WbcBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics &eeKinematics)
        : pinocchio_interface_measured_(pinocchioInterface),//数据接口用于测量实际值还有期望值
          pinocchio_interface_desired_(pinocchioInterface),
          info_(std::move(info)),//创建任务实例
          ee_kinematics_(eeKinematics.clone()),//末端执行器实例
          mapping_(info_),//广义坐标下的实例
          input_last_(vector_t::Zero(info_.inputDim)) {//上次输入信号的储存
        num_decision_vars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;//状态的维度
        q_measured_ = vector_t(info_.generalizedCoordinatesNum);//广义坐标的输入数 三维坐标的数量 还有受控关节数量
        v_measured_ = vector_t(info_.generalizedCoordinatesNum);
    }

    vector_t WbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired,
                             const vector_t &rbdStateMeasured, size_t mode,
                             scalar_t /*period*/) {
        contact_flag_ = modeNumber2StanceLeg(mode);
        num_contacts_ = 0;
        for (const bool flag: contact_flag_) {
            if (flag) {
                num_contacts_++;
            }
        }

        updateMeasured(rbdStateMeasured);
        updateDesired(stateDesired, inputDesired);

        return {};
    }

    void WbcBase::updateMeasured(const vector_t &rbdStateMeasured) {
        q_measured_.head<3>() = rbdStateMeasured.segment<3>(3);//基座位置：q_measured_.head<3>()
        q_measured_.segment<3>(3) = rbdStateMeasured.head<3>();//基座的欧拉角
        q_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);//关节角
        v_measured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);//基座的线速度
        v_measured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            q_measured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));//通过函数将基座角速度从全局角速度转换为 ZYX 欧拉角的时间导数
        v_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(
            info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);//更新测量到的 关节速度。

        const auto &model = pinocchio_interface_measured_.getModel();//获取模型
        auto &data = pinocchio_interface_measured_.getData();

        // For floating base EoM task
        forwardKinematics(model, data, q_measured_, v_measured_);//算每个关节的位置、速度以及全局参考系下的基座和末端位置
        computeJointJacobians(model, data);// 更新所有关节的雅可比矩阵，表示从关节速度到末端速度的关系。
        updateFramePlacements(model, data);//更新末端执行器的位姿信息
        crba(model, data, q_measured_);//计算质量矩阵
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();//确保矩阵质量M是对称的
        nonLinearEffects(model, data, q_measured_, v_measured_);//计算非线性项，比如重力什么的，保证轨迹跟踪的精确性
        j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED,
                             jac);//第 i 个末端的帧索引。
            j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }//    提取每个末端的雅可比矩阵 jac 并将其线性部分（topRows<3>()）存储到全局雅可比矩阵 j_。
        // For not contact motion task
        computeJointJacobiansTimeVariation(model, data, q_measured_, v_measured_);
        dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                          pinocchio::LOCAL_WORLD_ALIGNED, jac);//info_.endEffectorFrameIndices[i]：末端的帧索引，指示当前计算的足端
            dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();//pinocchio::LOCAL_WORLD_ALIGNED
         //计算全局末端雅可比矩阵的时间变化率可比矩阵的时间变化率描述了末端速度的动态变化，尤其是非恒定关节速度引起的二阶效应。
         //在动态环境下（如加速步态或跳跃），需要考虑 J˙ 的影响以确保轨迹精度。
        }
    }

    void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired) {
        const auto &model = pinocchio_interface_desired_.getModel();
        auto &data = pinocchio_interface_desired_.getData();

        mapping_.setPinocchioInterface(pinocchio_interface_desired_);
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);/// 使用 mapping_ 将目标状态 stateDesired 映射到广义坐标
        forwardKinematics(model, data, qDesired);//： 更新每个连杆的位姿（位置和方向）
        computeJointJacobians(model, data, qDesired);// 计算各关节的雅可比矩阵，用于后续的末端控制。
        updateFramePlacements(model, data);// 更新每个关节或末端在运动学中的具体位姿
        updateCentroidalDynamics(pinocchio_interface_desired_, info_, qDesired);//updateCentroidalDynamics 全局动力学信息
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);//目标状态和输入映射到目标速度空间 
        forwardKinematics(model, data, qDesired, vDesired);//使用 qDesiredqDesired​ 和 vDesiredvDesired​ 更新运动学信息，包括末端速度等。
    }

    Task WbcBase::formulateFloatingBaseEomTask() {
        const auto &data = pinocchio_interface_measured_.getData();

        matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
        s.block(0, 0, info_.actuatedDofNum, 6).setZero();
        s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

        matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, num_decision_vars_) << data.M, -j_.transpose(), -s.
                      transpose()).finished();
        vector_t b = -data.nle;//非线性效应向量，包括 Coriolis 和重力项。

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateTorqueLimitsTask() {
        matrix_t d(2 * info_.actuatedDofNum, num_decision_vars_);// // 创建任务矩阵 d，用于存储力矩约束的约束矩阵。
        d.setZero();
        matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);//// 创建单位矩阵 i，表示每个力矩单独限制
        d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
                info_.actuatedDofNum) = i;//赋值几个力矩
        d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts,
                info_.actuatedDofNum,
                info_.actuatedDofNum) = -i;
        vector_t f(2 * info_.actuatedDofNum);//创建f储存限制值
        for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
            f.segment<3>(3 * l) = torque_limits_;// 设置力矩上下限的具体值。
        }
        return {matrix_t(), vector_t(), d, f}; // 返回无等式约束（a,b），只包含不等式约束（d,f）。
    } //等式约束问题指，在设计变量空间向量fx 内，最小化：同时满足等式约束向量cx=0 ，其中约束的个数m要不能大于变量空间x中变量的个数n，即 m<n
 //不等式指的是m可以>n
    Task WbcBase::formulateNoContactMotionTask() {
        matrix_t a(3 * num_contacts_, num_decision_vars_);// 创建等式约束矩阵 a
        vector_t b(a.rows()); // 创建等式约束向量 b。
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
            if (contact_flag_[i]) { // 检查接触状态标志。
               // 将当前接触点的雅可比矩阵添加到约束矩阵 a。
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum);//将当前加速度添加到约束矩阵b
                b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateFrictionConeTask() {
        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        a.setZero();
        size_t j = 0;// 对非接触点设置约束。
        // 设置非接触点在加速度约束中的矩阵块。
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            if (!contact_flag_[i]) {
                a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
            }
        }
        vector_t b(a.rows());// 等式约束向量的维度
        b.setZero();

        matrix_t frictionPyramic(5, 3); // clang-format off / 定义摩擦锥的锥形矩阵。
        frictionPyramic << 0, 0, -1, // z方向力的负值。
                   1, 0, -friction_coeff_,  // x正方向摩擦约束。
                  -1, 0, -friction_coeff_,  // x负方向摩擦约束。
                   0, 1, -friction_coeff_,  // y正方向摩擦约束。
                   0, -1, -friction_coeff_; // y负方向摩擦约束。

        matrix_t d(5 * num_contacts_ + 3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        d.setZero();// 初始化为零。
        j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            if (contact_flag_[i]) {//为接触点设置摩擦锥不等式约束。
                d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
            } // 初始化不等式约束的偏移量。
        }
        vector_t f = Eigen::VectorXd::Zero(d.rows());

        return {a, b, d, f}; // 返回等式和不等式约束。
    }

    Task WbcBase::formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period) {
        matrix_t a(6, num_decision_vars_); // 等式约束矩阵。
        a.setZero();
        a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);// 设置基座的加速度矩阵

        vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - input_last_, info_) / period;
        input_last_ = inputDesired;//// 更新最近输入。
        mapping_.setPinocchioInterface(pinocchio_interface_desired_);

        const auto &model = pinocchio_interface_desired_.getModel();
        auto &data = pinocchio_interface_desired_.getData();
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);//映射目标状态
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
            /// 映射目标速度。
        const auto &A = getCentroidalMomentumMatrix(pinocchio_interface_desired_);// 获取质心动量矩阵。
        const Matrix6 Ab = A.template leftCols<6>();//// 提取关于基座相关矩阵。
        const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);//计算矩阵逆。
        const auto Aj = A.rightCols(info_.actuatedDofNum);// // 提取关节相关矩阵
        const auto ADot = dccrba(model, data, qDesired, vDesired);// // 计算动量变化率。
        Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(
                                             pinocchio_interface_desired_, info_, inputDesired);
        centroidalMomentumRate.noalias() -= ADot * vDesired; // 动量变化率减去基座速度的影响。
        centroidalMomentumRate.noalias() -= Aj * jointAccel;// 减去关节加速度的影响。

        Vector6 b = AbInv * centroidalMomentumRate;// 计算等式约束的右端项。

        return {a, b, matrix_t(), vector_t()}; // 返回等式约束。
    }

    Task WbcBase::formulateSwingLegTask() {
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);// 使用当前测量的机器人状态初始化逆运动学接口
        std::vector<vector3_t> posMeasured = ee_kinematics_->getPosition(vector_t());//获取当前末端关节的实际位置
        std::vector<vector3_t> velMeasured = ee_kinematics_->getVelocity(vector_t(), vector_t());//速度
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_desired_); // 使用目标状态初始化逆运动学接口
        std::vector<vector3_t> posDesired = ee_kinematics_->getPosition(vector_t());   // 获取末端关节的目标位置
        std::vector<vector3_t> velDesired = ee_kinematics_->getVelocity(vector_t(), vector_t());//目标速度

        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        vector_t b(a.rows());// 初始化线性约束矩阵a和目标向量b
        a.setZero();
        b.setZero();
        size_t j = 0;// 记录未接触点的索引
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            if (!contact_flag_[i]) {
                vector3_t accel = swing_kp_ * (posDesired[i] - posMeasured[i]) + swing_kd_ * (
                                      velDesired[i] - velMeasured[i]);
            // 填充未接触点的运动学雅可比矩阵
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum);
                       // 填充加速度误差项
                b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
                j++;// 未接触点索引增加
            }
        }
        //返回等式约束
        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateContactForceTask(const vector_t &inputDesired) const {
        matrix_t a(3 * info_.numThreeDofContacts, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
    // 初始化线性约束矩阵a和目标向量b
        // 为每个接触点构建力约束矩阵
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }// 在决策变量中设置接触力项
           // 目标向量b设置为输入期望力
        b = inputDesired.head(a.rows());
    // 返回构造的接触力任务
        return {a, b, matrix_t(), vector_t()};
    }

    void WbcBase::loadTasksSetting(const std::string &taskFile, const bool verbose) {
        // Load task file
        torque_limits_ = vector_t(info_.actuatedDofNum / 4);
        loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torque_limits_);
        if (verbose) {
            std::cerr << "\n #### Torque Limits Task:";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << "\n #### Hip_joint Thigh_joint Calf_joint: " << torque_limits_.transpose() << "\n";
            std::cerr << " #### =============================================================================\n";
        }
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        std::string prefix = "frictionConeTask.";
        if (verbose) {
            std::cerr << "\n #### Friction Cone Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, friction_coeff_, prefix + "frictionCoefficient", verbose);
        if (verbose) {
            std::cerr << " #### =============================================================================\n";
        }
        prefix = "swingLegTask.";
        if (verbose) {
            std::cerr << "\n #### Swing Leg Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, swing_kp_, prefix + "kp", verbose);
        loadData::loadPtreeValue(pt, swing_kd_, prefix + "kd", verbose);
    }
} // namespace legged
