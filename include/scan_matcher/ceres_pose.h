#pragma once

#include <array>
#include <memory>

#include "Eigen/Core"
#include "transform.h"
#include "ceres/ceres.h"
#include "ceres/solver.h"

namespace optimization{
    class CeresPose{
    public:
        CeresPose(const transform::Rigid3d &rigid,
                  std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
                  std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
                  ceres::Problem *problem);

        const transform::Rigid3d ToRigid() const;
        double* translation() { return data_->translation.data(); }
        const double* translation() const { return data_->translation.data(); }

        double* rotation() { return data_->rotation.data(); }
        const double* rotation() const { return data_->rotation.data(); }

        struct Data {
             std::array<double, 3> translation;
             // Rotation quaternion as (w, x, y, z).
            std::array<double, 4> rotation;
        };

        Data& data() { return *data_; }

        private:
        std::shared_ptr<Data> data_;
    };
}