// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "context-libusb.h"
#include "../types.h"

namespace librealsense
{
    namespace platform
    {       
        usb_context::usb_context() : _ctx(NULL), _list(NULL), _count(0)
        {
            LOG_INFO("Attempting libusb_init...");
            try {
                auto sts = libusb_init(&_ctx);
                LOG_INFO("libusb_init returned status: " << sts);
                if(sts != LIBUSB_SUCCESS)
                {
                    LOG_ERROR("libusb_init failed with status: " << sts);
                    _ctx = nullptr;
                    _list = nullptr;
                    _count = 0;
                }
                else
                {
                    LOG_INFO("libusb_init succeeded, getting device list...");
                    _count = libusb_get_device_list(_ctx, &_list);
                    LOG_INFO("Found " << _count << " USB devices");
                }
            }
            catch(const std::exception& e) {
                LOG_ERROR("Exception during libusb_init: " << e.what());
                _ctx = nullptr;
                _list = nullptr;
                _count = 0;
            }
            catch(...) {
                LOG_ERROR("Unknown exception during libusb_init");
                _ctx = nullptr;
                _list = nullptr;
                _count = 0;
            }
        }
        
        usb_context::~usb_context()
        {
            if (_list)
                libusb_free_device_list(_list, true);
            assert(_handler_requests == 0); // we need the last libusb_close to trigger an event to stop the event thread
            if (_event_handler.joinable())
                _event_handler.join();
            if (_ctx)
                libusb_exit(_ctx);
        }
        
        libusb_context* usb_context::get()
        {
            return _ctx;
        } 
    
        void usb_context::start_event_handler()
        {
            if (!_ctx) return; // Skip if libusb initialization failed
            
            std::lock_guard<std::mutex> lk(_mutex);
            if (!_handler_requests) {
                // see "Applications which do not use hotplug support" in libusb's io.c
                if (_event_handler.joinable()) {
                    _event_handler.join();
                    _kill_handler_thread = 0;
                }
                _event_handler = std::thread([this]() {
                    while (!_kill_handler_thread)
                        libusb_handle_events_completed(_ctx, &_kill_handler_thread);
                });
            }
            _handler_requests++;
        }

        void usb_context::stop_event_handler()
        {
            std::lock_guard<std::mutex> lk(_mutex);
            _handler_requests--;
            if (!_handler_requests)
                // the last libusb_close will trigger and event and the handler thread will notice this is set
                _kill_handler_thread = 1;
        }

        libusb_device* usb_context::get_device(uint8_t index)
        {
            return index < _count ? _list[index] : NULL;
        }
        
        size_t usb_context::device_count()
        {
            return _count;
        }
    }
}
