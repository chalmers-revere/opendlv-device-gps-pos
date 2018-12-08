/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef POS_DECODER
#define POS_DECODER

#include "opendlv-standard-message-set.hpp"
#include "pos-message-set.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <string>

class POSDecoder {
   private:
        enum GRP_SIZES {
            BUFFER_SIZE                 = 2048,
            GRP_HEADER_SIZE             = 8,
            GRP_FOOTER_SIZE             = 4,
            TIME_DISTANCE_FIELD_SIZE    = 26,
        };

        enum POSMessages {
            UNKNOWN                     = 0,
            GRP1                        = 1,
            GRP2                        = 2,
            GRP3                        = 3,
            GRP4                        = 4,
            GRP10001                    = 10001,
            GRP10002                    = 10002,
            GRP10003                    = 10003,
            GRP10009                    = 10009,
        };

   private:
    POSDecoder(const POSDecoder &) = delete;
    POSDecoder(POSDecoder &&)      = delete;
    POSDecoder &operator=(const POSDecoder &) = delete;
    POSDecoder &operator=(POSDecoder &&) = delete;

   public:
    POSDecoder(std::function<void(const double &latitude, const double &longitude, const cluon::data::TimeStamp &sampleTime)> delegateLatitudeLongitude,
                std::function<void(const float &heading, const cluon::data::TimeStamp &sampleTime)> delegateHeading) noexcept;
    ~POSDecoder();

   public:
    void decode(const std::string &data, std::chrono::system_clock::time_point &&tp) noexcept;

   private:
    size_t parseBuffer(uint8_t *buffer, const size_t size, std::chrono::system_clock::time_point &&tp);

   private:
    std::function<void(const double &latitude, const double &longitude, const cluon::data::TimeStamp &sampleTime)> m_delegateLatitudeLongitude{};
    std::function<void(const float &heading, const cluon::data::TimeStamp &sampleTime)> m_delegateHeading{};

   private:
    opendlv::device::gps::pos::TimeDistance getTimeDistance(std::stringstream &buffer);
    opendlv::device::gps::pos::Grp1Data getGRP1(std::stringstream &buffer);
    opendlv::device::gps::pos::Grp2Data getGRP2(std::stringstream &buffer);
    opendlv::device::gps::pos::Grp3Data getGRP3(std::stringstream &buffer);
    opendlv::device::gps::pos::GNSSReceiverChannelStatus getGNSSReceiverChannelStatus(std::stringstream &buffer);
    opendlv::device::gps::pos::Grp4Data getGRP4(std::stringstream &buffer);
    opendlv::device::gps::pos::Grp10001Data getGRP10001(std::stringstream &buffer, uint32_t payloadSize);
    opendlv::device::gps::pos::Grp10002Data getGRP10002(std::stringstream &buffer, uint32_t payloadSize);
    opendlv::device::gps::pos::Grp10003Data getGRP10003(std::stringstream &buffer);
    opendlv::device::gps::pos::Grp10009Data getGRP10009(std::stringstream &buffer, uint32_t payloadSize);

   private:
    uint8_t *m_dataBuffer{nullptr};
    size_t m_size{0};
    int64_t m_timeOffsetSinceGPSinMicroseconds{0};
};

#endif

