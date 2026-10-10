
/**
 * @file
 * @brief Header file containing declarations for advanced Dev Features of the Sixdof Falcon library.
 *
 * This file contains the tools needed to customize the Sixdof Falcon tracking system parameters.
 * Before changing parameters please contact the Sixdof team!
 */

#pragma once
#include "SdsFalcon.h"

namespace Sds {
    namespace Falcon {

        // name, default, min, max, description
        #define DEV_FEATURE_LIST(F) \
            F(std_xyz,                    0.005,    0.0,  1.0,  "EKF process noise x,y,z (m)") \
            F(std_xyz_f,                  0.5,      0.0,  1.0,  "EKF process noise x,y,z factor (%)") \
            F(damp_factor_ms,             0.46,     0.0,  1.0,  "Ekf velocity dampening factor (ms)") \
            F(sigma_gyro_offset,          0.001,    0.0,  1.0,  "Gyro noise (deg)") \
            F(sigma_gyro_factor,          0.005,    0.0,  1.0,  "Gyro noise factor (%)") \
            F(sigma_gyro_offset_unsolved, 0.06,     0.0,  1.0,  "Ekf gyro noise for when the gyro bias hasn't been solved yet (deg)") \
            F(sigma_gyro_factor_unsolved, 0.05,     0.0,  1.0,  "Ekf gyro noise factor for when the gyro bias hasn't been solved yet (%)") \
            F(use_damp_factor,            1.0,      0.0,  1.0,  "Ekf use dampening factor") \
            F(use_2d_extrapolation,       1.0,      0.0,  1.0,  "Ekf use 2d extrapolation for high elevation peaks") \
            F(process_noise_idx,          0.3,      0.0,  1.0,  "Peak process noise (pixel)") \
            F(process_noise_v,            0.004625, 0.0,  1.0,  "Peak process noise velocity (pixel / frame)") \
            F(meas_noise,                 0.5,      0.0,  1.0,  "Peak measurement noise (pixel)") \
            F(localization_cooldown_sec,  0.02,     0.0,  1.0,  "Localization retry cooldown (sec)") \
            F(localization_features,      3.0,      3.0,  10.0, "P3P Localization minimum number of features observed")\

        /**
         * @enum ParamType
         * @brief An enum representing the parameters that can be set by DevFeatures
         *
         * An enum representing the parameters that can be set by DevFeatures.
         */
        enum class ParamType : int {
            Unknown = 0, 
            #define MAKE_PARAM_TYPE_ENUM(name, default, min, max, description) name,
            DEV_FEATURE_LIST(MAKE_PARAM_TYPE_ENUM)
            #undef MAKE_PARAM_TYPE_ENUM
        };


        /**
         * @class DevFeatures
         * @brief Class for managing runtime parameters of the Sixdof Falcon tracking system.
         *
         * This class provides an interface for managing the runtime parameters of the Sixdof Falcon tracking system.
         */
        class SDS_FALCON_API DevFeatures { 
        public:
            virtual ~DevFeatures() { }

            /**
             * @brief Sets a parameter.
             */
            virtual void setParam(ParamType, double) = 0;

            /**
             * @brief Gets the current value of a parameter.
             */
            virtual double getParam(ParamType) = 0;
        };

        /**
         * @brief Create an instance of DevFeatures class.
         * 
         * @return A pointer to a DevFeatures class.
         */
        SDS_FALCON_API DevFeatures* createDevFeatures();

        /**
         * @brief Load an XML file with the Development Features.
         * 
         * @return False if the load was unsuccessful, true if successful
         */
        SDS_FALCON_API bool loadDevFeatures(DevFeatures* devFeatures, const std::string& path);

        /**
         * @brief Save an XML file with the Development Features.
         */
        SDS_FALCON_API void saveDevFeatures(DevFeatures* devFeatures, const std::string& path);

        /**
         * @brief Load the Development Features with their default values.
         */
        SDS_FALCON_API void loadDefaultDevFeatures(DevFeatures* devFeatures);
    }
}