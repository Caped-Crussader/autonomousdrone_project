/**
 * @file
 * @brief Header file containing declarations relating to the Sixdof sensors shutter control.
 *
 * This file contains the declarations for classes and data structures related to controlling the shutter of the Sixdof sensor.
 * This header file is shared between the Sixdof Falcon, and Sixdof Cheetah software products.
 * 
 * The functionality provided in this file is optional, the default shutter behaviour will work for most use cases.
 * Users with more complicated deployment scenarios, such as cases where; 
 * there are LEDs of different brightnesses, with very fast moving targets, or situations with complicated lighting features, may use the functionality presented here to develop custom shutter control logic.
 *
 * The Sixdof sensor has 17 shutter values, 0 to 16, where 0 is the minimum shutter exposer time, and 16 is the maximum shutter exposure time.
 * Additionally, there is a mode called "Auto-shutter" which allows the sensor itself to optimize the shutter.\
 * Auto-shutter mode is acceptable in many use cases, however it does not use any information other than the raw data to set the shutter value. 
 * This limitation makes Auto-shutter a poor choice in environments where there may be lots of background interference, due to sunlight.
 * 
 * The Sixdof Falcon, and Sixdof Cheetah, solutions both provide default shutter behaviour that take into consideration more information,
 * such as the beacon ids that are currently being used. Therefore, the default shutter behaviour preforms better than Auto-shutter in many circumstances.
 * 
 * The following classes enable the user to override the default shutter behaviour for advanced deployment use cases.
 * 
 * In order to customize the shutter logic, inherit from the IShutterControl base class. Then the setShutterAuto, and setShutterFixed functions can be used to change the shutter.
 * Note, it does take a small period of time for the sensor to change its shutter, so these functions should not be called more than every 20ms. 
 * 
 * Both SdsFalcon and SdsCheetah can accept a std::shared_ptr<IShutterControl>.
 * Additionally, the custom shutter logic can be compiled into a plugin that can be compiled and provided to SdsFalcon or SdsCheetah.
 * The advantage of a compiled plugin is that it can be loaded into the Sixdof Falcon Studio, or Sixdof Cheetah Studio GUI applications for testing.
 * To make a plugin implement a shared library with the following fuction:
 * extern "C" IShutterControl* createShutterControl()
 */

#pragma once
#include <stdint.h>
#include <vector>
#include <memory>

#ifdef _WIN32
    // Windows platform
    #ifdef SDS_EXPORTS
        #define SDS_API __declspec(dllexport)
    #else
        #define SDS_API __declspec(dllimport)
    #endif
#else
    // Non-Windows platform
    #define SDS_API __attribute__((visibility("default")))
#endif

namespace Sds {

    /**
     * @class ISetShutterListener
     * @brief Listener interface that allows IShutterControl to pass forward requests to change the shutter
     * 
     * This class is used to listen to shutter change requests to SdsFalcon and SdsCheetah. 
     * For all intensive purposes this class can be ignored.
     */
    class SDS_API ISetShutterListener {
    public:
        virtual void setShutterAuto(uint8_t min_shutter, uint8_t max_shutter) = 0;

        virtual void setShutterFixed(uint8_t shutter) = 0;
    };

    /**
     * @struct OpticalPeak
     * @brief A structure representing a summary of the optical data for a single beacon
     *
     * This data represents a summary of the optical data corresponding to to a single beacon seen by a single sensor.
     * This data can be used to customize shutter control.
     */
    struct OpticalPeak {
        bool has_id;        ///< peak has a parsed beacon id.
        bool id_duplicate;  ///< this peak has a repeat id, usually indicating a reflection. 
        uint32_t id;        ///< the beacon id.
        double height;      ///< the intensity of the peak [0, 255].
        double background;  ///< a sample of the intensity of the background in the locality of the peak.
        char carnea;        ///< the id of the carnea sensor ['C', 'L', 'P'].
    };

    /**
     * @class IShutterControl
     * @brief Shutter control base class
     * 
     * Inherit from this base class to create custom shutter control logic.
     */
    class SDS_API IShutterControl {
    public:

        virtual ~IShutterControl() = default;
                
        /**
         * @brief Custom logic to update the shutter.
         *
         * override this function with your custom logic,
         * call setShutterAuto or setShutterFixed to change the shutter
         * only call setShutterAuto or setShutterFixed when you need the shutter control to change
         * avoid making shutter changes within a small period of time, otherwise some of the requests may be ignored by the sensor
         */
        virtual void updateShutter(uint8_t currentShutter, const std::vector<OpticalPeak>& peaks) = 0;

        /**
         * @brief Set the shutter to Auto-shutter.
         * Use this function to set the shutter to auto shutter,
         * min and max shutter values must be in the range [0, 16].
         */
        void setShutterAuto(uint8_t min_shutter, uint8_t max_shutter) {
            if (listener) { listener->setShutterAuto(min_shutter, max_shutter); }
        }

        /**
         * @brief Set the shutter to a fixed value
         * 
         * Use this function to set the shutter to a fixed shutter value,
         * shutter value must be in the range [0, 16].
         */
        void setShutterFixed(uint8_t shutter) {
            if (listener) { listener->setShutterFixed(shutter); }
        }

        /**
         * @brief Register a shutter listener
         * 
         * This function can be ignored by the client.
         */
        void registerSetShutterListener(ISetShutterListener* l) {
            listener = l;
        };
    private:
        ISetShutterListener* listener;
    };

}