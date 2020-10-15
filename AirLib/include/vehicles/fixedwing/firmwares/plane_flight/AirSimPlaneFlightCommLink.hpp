// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimPlaneFlightCommLink_hpp
#define msr_airlib_AirSimPlaneFlightCommLink_hpp

#include <exception>
#include "firmware/interfaces/ICommLink.hpp"
#include "common/Common.hpp"


namespace msr { namespace airlib {


class AirSimPlaneFlightCommLink : public plane_flight::ICommLink {
public: // derived class specific methods
    void getStatusMessages(std::vector<std::string>& messages)
    {
        if (messages_.size() > 0) {
            messages.insert(messages.end(), messages_.begin(), messages_.end());
            messages_.clear();
        }
    }

public: // implement CommLink interface
    virtual void reset()
    {
        plane_flight::ICommLink::reset();

        messages_.clear();
    }

    virtual void update()
    {
        plane_flight::ICommLink::update();
    }

    virtual void log(const std::string& message, int32_t log_level = ICommLink::kLogLevelInfo)
    {
        unused(log_level);
        //if (log_level > 0)
        //    Utils::DebugBreak();
        messages_.push_back(std::string(message));
    }

private:
    std::vector<std::string> messages_;

};


}} //namespace
#endif
