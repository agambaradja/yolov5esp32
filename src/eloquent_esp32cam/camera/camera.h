#ifndef ELOQUENT_ESP32CAMERA_CAMERA_CAMERA
#define ELOQUENT_ESP32CAMERA_CAMERA_CAMERA
/*
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define pins_d0 = 15;
#define pins_d1 = 17;
#define pins_d2 = 18;
#define pins_d3 = 16;
#define pins_d4 = 14;
#define pins_d5 = 12;
#define pins_d6 = 11;
#define pins_d7 = 48;
#define pins_xclk = 10;
#define pins_pclk = 13;
#define pins_vsync = 38;
#define pins_href = 47;
#define pins_sccb_sda = 40;
#define pins_sccb_scl = 39;
#define pins_pwdn = -1;
#define pins_reset = -1;
#define pins_flashlight = -1;
*/
#include <esp_camera.h>
#include "./quality.h"
#include "./brownout.h"
#include "./xclk.h"
#include "./resolution.h"
#include "./pinout.h"
#include "./sensor.h"
#include "./pixformat.h"
#include "./rgb_565.h"
#include "../extra/exception.h"
#include "../extra/time/rate_limit.h"
#include "../extra/esp32/multiprocessing/mutex.h"

using Eloquent::Error::Exception;
using Eloquent::Extra::Esp32::Multiprocessing::Mutex;
using Eloquent::Extra::Time::RateLimit;

namespace Eloquent
{
    namespace Esp32cam
    {
        namespace Camera
        {
            /**
             * Configure and use the camera,
             * Eloquent style
             */
            class Camera
            {
            public:
                camera_config_t config;
                camera_fb_t *frame;
                JpegQuality quality;
                Brownout brownout;
                XCLK xclk;
                Resolution resolution;
                Sensor sensor;
                Pixformat pixformat;
                Exception exception;
                RateLimit rateLimit;
                Mutex mutex;
                Converter565<Camera> rgb565;
                
                /**
                 * Constructor
                 */
                Camera() : exception("Camera"),
                           mutex("Camera"),
                           rgb565(this)
                {
                }

                /**
                 *
                 * @return
                 */
                Exception &begin()
                {
                    // assign pins
                    camera_config_t config;
                    config.ledc_channel = LEDC_CHANNEL_0;
                    config.ledc_timer = LEDC_TIMER_0;
                    config.pin_d0 = 15;
                    config.pin_d1 = 17;
                    config.pin_d2 = 18;
                    config.pin_d3 = 16;
                    config.pin_d4 = 14;
                    config.pin_d5 = 12;
                    config.pin_d6 = 11;
                    config.pin_d7 = 48;
                    config.pin_xclk = 10;
                    config.pin_pclk = 13;
                    config.pin_vsync = 38;
                    config.pin_href = 47;
                    config.pin_sccb_sda = 40;
                    config.pin_sccb_scl = 39;
                    config.pin_pwdn = -1;
                    config.pin_reset = -1;
                    config.xclk_freq_hz = 8 * 1000000;
                    config.pixel_format = PIXFORMAT_RGB565;

                    config.frame_size = resolution.framesize;
                    config.fb_location = CAMERA_FB_IN_PSRAM;
                    config.fb_count = 2;
                    config.grab_mode = CAMERA_GRAB_LATEST;

                    // Camera init
                    esp_err_t err = esp_camera_init(&config);
                    if (err != ESP_OK)
                        return exception.set("Camera init failed");
                    

                    sensor.setFrameSize(resolution.framesize);

                    return exception.clear();
                }

                /**
                 * Capture new frame
                 */
                Exception &capture()
                {
                    if (!rateLimit)
                        return exception.soft().set("Too many requests for frame");

                    mutex.threadsafe([this]()
                                     {
                            free();
                            frame = esp_camera_fb_get(); },
                                     1000);

                    if (!mutex.isOk())
                        return exception.set("Cannot acquire mutex");

                    rateLimit.touch();

                    if (!hasFrame())
                        return exception.set("Cannot capture frame");

                    return exception.clear();
                }

                /**
                 * Release frame memory
                 */
                void free()
                {
                    if (frame != NULL)
                    {
                        esp_camera_fb_return(frame);
                        frame = NULL;
                    }
                }

                /**
                 * Test if camera has a valid frame
                 */
                inline bool hasFrame() const
                {
                    return frame != NULL && frame->len > 0;
                }

                /**
                 * Get frame size in bytes
                 * @return
                 */
                inline size_t getSizeInBytes()
                {
                    return hasFrame() ? frame->len : 0;
                }

                /**
                 * Save to given folder with automatic name
                 */
                template <typename Disk>
                Exception &saveTo(Disk &disk, String folder = "")
                {
                    return saveToAs(disk, folder, "");
                }

                /**
                 * Save to given folder with given name
                 */
                template <typename Disk>
                Exception &saveToAs(Disk &disk, String folder = "", String filename = "")
                {
                    if (!hasFrame())
                        return exception.set("No frame to save");

                    if (filename == "")
                        filename = disk.getNextFilename("jpg");

                    if (folder != "")
                        filename = folder + '/' + filename;

                    if (filename[0] != '/')
                        filename = String('/') + filename;

                    return disk.writeBinary(filename, frame->buf, frame->len);
                }

            protected:
            };
        }
    }
}

namespace eloq
{
    static Eloquent::Esp32cam::Camera::Camera camera;
}

#endif