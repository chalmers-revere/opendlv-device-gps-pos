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

#include "cluon-complete.hpp"
#include "pos-decoder.hpp"

#include <cmath>
#include <cstring>
#include <ctime>
#include <array>
#include <sstream>
#include <string>

POSDecoder::POSDecoder(std::function<void(const double &latitude, const double &longitude, const cluon::data::TimeStamp &sampleTime)> delegateLatitudeLongitude,
                       std::function<void(const float &heading, const cluon::data::TimeStamp &sampleTime)> delegateHeading,
                       std::function<void(opendlv::device::gps::pos::Grp1Data d, const cluon::data::TimeStamp &sampleTime)> delegateGrp1Data,
                       std::function<void(opendlv::device::gps::pos::Grp2Data d, const cluon::data::TimeStamp &sampleTime)> delegateGrp2Data,
                       std::function<void(opendlv::device::gps::pos::Grp3Data d, const cluon::data::TimeStamp &sampleTime)> delegateGrp3Data,
                       std::function<void(opendlv::device::gps::pos::Grp4Data d, const cluon::data::TimeStamp &sampleTime)> delegateGrp4Data) noexcept
    : m_delegateLatitudeLongitude(std::move(delegateLatitudeLongitude))
    , m_delegateHeading(std::move(delegateHeading))
    , m_delegateGrp1Data(std::move(delegateGrp1Data))
    , m_delegateGrp2Data(std::move(delegateGrp2Data))
    , m_delegateGrp3Data(std::move(delegateGrp3Data))
    , m_delegateGrp4Data(std::move(delegateGrp4Data)) {
    m_dataBuffer = new uint8_t[POSDecoder::BUFFER_SIZE];

    // Calculate offset between GPS and UTC.
    auto createTimeFromYYYYMMDD = [](int year, int month, int day){
        struct tm tm;
        std::memset(&tm, 0, sizeof(struct tm));
        tm.tm_year = year - 1900;
        tm.tm_mon = month - 1;
        tm.tm_mday = day;
        return mktime(&tm);
    };

    time_t currentTime;
    std::memset(&currentTime, 0, sizeof(time_t));
    std::time(&currentTime);

    struct tm currentTimeBrokenDown;
    std::memset(&currentTimeBrokenDown, 0, sizeof(struct tm));
    gmtime_r(&currentTime, &currentTimeBrokenDown);

    const constexpr int32_t GPS_EPOCH_OFFSET{315964800};
    const constexpr int32_t GPS_LEAP_SECONDS{-18};
    constexpr const int64_t SECONDS_PER_WEEK{60*60*24*7};
    const double diff = difftime(createTimeFromYYYYMMDD(currentTimeBrokenDown.tm_year+1900, currentTimeBrokenDown.tm_mon+1, currentTimeBrokenDown.tm_mday), createTimeFromYYYYMMDD(1980, 1, 6));
    m_timeOffsetSinceGPSinMicroseconds = (static_cast<int64_t>(diff)/SECONDS_PER_WEEK)*SECONDS_PER_WEEK + GPS_EPOCH_OFFSET + GPS_LEAP_SECONDS;
    m_timeOffsetSinceGPSinMicroseconds *= static_cast<int64_t>(1000*1000);
}

POSDecoder::~POSDecoder() {
    delete [] m_dataBuffer;
    m_dataBuffer = nullptr;
}

void POSDecoder::decode(const std::string &data, std::chrono::system_clock::time_point &&tp) noexcept {
    const size_t bytesAvailable{data.size()};
    size_t bytesCopied{0};

    while (bytesCopied != bytesAvailable) {
        // How many bytes can we store in our buffer?
        size_t bytesToCopy{((POSDecoder::BUFFER_SIZE - m_size) < bytesAvailable) ? (POSDecoder::BUFFER_SIZE - m_size) : bytesAvailable};
        std::memcpy(m_dataBuffer+m_size, data.data(), bytesToCopy);
        // Store how much we copied.
        bytesCopied += bytesToCopy;
        // Booking for the m_buffer fill level.
        m_size += bytesToCopy;
        // Consume data from m_buffer.
        size_t consumed = parseBuffer(m_dataBuffer, m_size, std::move(tp));
        // Discard processed entries.
        for (size_t i{0}; (consumed > 0) && (i < (m_size - consumed)); i++) {
            m_dataBuffer[i] = m_dataBuffer[i + consumed];
        }
        m_size -= consumed;
        // If the parser does not work at all, cancel it.
        if (m_size >= POSDecoder::BUFFER_SIZE) {
            break;
        }
    }
}

size_t POSDecoder::parseBuffer(uint8_t *buffer, const size_t size, std::chrono::system_clock::time_point &&tp) {
    cluon::data::TimeStamp sampleTimeStamp{cluon::time::convert(std::move(tp))};

    auto extractTimeDistance = [timeOffsetSinceGPSinMicroseconds = m_timeOffsetSinceGPSinMicroseconds](double time1){
        int64_t seconds{static_cast<int64_t>(floor(time1))};
        int64_t microseconds{static_cast<int64_t>(floor((time1-seconds)* static_cast<int64_t>(1000*1000)))};

        cluon::data::TimeStamp relativeTimeStamp;
        relativeTimeStamp.seconds(seconds).microseconds(microseconds);

        return cluon::time::fromMicroseconds(timeOffsetSinceGPSinMicroseconds + cluon::time::toMicroseconds(relativeTimeStamp));
    };

    size_t offset{0};
    while (true) {
        // Sanity check whether we consumed all data.
        if ((offset + POSDecoder::GRP_HEADER_SIZE) > size) {
            return offset;
        }
        if ( ('$' == buffer[offset + 0]) &&
             ('G' == buffer[offset + 1]) &&
             ('R' == buffer[offset + 2]) &&
             ('P' == buffer[offset + 3]) ) {
            union {
                uint8_t array[2];
                uint16_t number{0};
            } twoBytes;

            // Next, read group number.
            uint16_t groupNumber{0};
            {
              twoBytes.array[0] = buffer[offset + 4];
              twoBytes.array[1] = buffer[offset + 5];
              groupNumber = le16toh(twoBytes.number);
            }

            // Next, read message size.
            uint16_t messageSize{0};
            {
              twoBytes.array[0] = buffer[offset + 6];
              twoBytes.array[1] = buffer[offset + 7];
              messageSize = le16toh(twoBytes.number);
            }

            // Check whether we can decode the next message.
            if ((offset + POSDecoder::GRP_HEADER_SIZE + messageSize) > size) {
                // We have a partial header and hence, need to wait for more data.
                return offset;
            }
            else {
                // We can decode the next message.
                const std::string message(reinterpret_cast<char*>(buffer + offset + POSDecoder::GRP_HEADER_SIZE), messageSize);
                std::stringstream b(message);

                if (POSDecoder::GRP1 == groupNumber) {
                    // Decode Applanix GRP1.
                    opendlv::device::gps::pos::Grp1Data g1Data{getGRP1(b)};

                    // Use timestamp from GPS if available.
                    if (1 == g1Data.timeDistance().time1Type()) {
                        sampleTimeStamp = extractTimeDistance(g1Data.timeDistance().time1());
                    }

                    if (nullptr != m_delegateLatitudeLongitude) {
                        m_delegateLatitudeLongitude(g1Data.lat(), g1Data.lon(), sampleTimeStamp);
                    }
                    if (nullptr != m_delegateHeading) {
                        m_delegateHeading(static_cast<float>(g1Data.heading()), sampleTimeStamp);
                    }
                    if (nullptr != m_delegateGrp1Data) {
                        m_delegateGrp1Data(g1Data, sampleTimeStamp);
                    }
                }
                else if (POSDecoder::GRP2 == groupNumber) {
                    // Decode Applanix GRP2.
                    opendlv::device::gps::pos::Grp2Data g2Data{getGRP2(b)};

                    // Use timestamp from GPS if available.
                    if (1 == g2Data.timeDistance().time1Type()) {
                        sampleTimeStamp = extractTimeDistance(g2Data.timeDistance().time1());
                    }

                    if (nullptr != m_delegateGrp2Data) {
                        m_delegateGrp2Data(g2Data, sampleTimeStamp);
                    }
                }
                else if (POSDecoder::GRP3 == groupNumber) {
                    // Decode Applanix GRP3.
                    opendlv::device::gps::pos::Grp3Data g3Data{getGRP3(b)};

                    // Use timestamp from GPS if available.
                    if (1 == g3Data.timeDistance().time1Type()) {
                        sampleTimeStamp = extractTimeDistance(g3Data.timeDistance().time1());
                    }

                    if (nullptr != m_delegateGrp3Data) {
                        m_delegateGrp3Data(g3Data, sampleTimeStamp);
                    }
                }
                else if (POSDecoder::GRP4 == groupNumber) {
                    // Decode Applanix GRP4.
                    opendlv::device::gps::pos::Grp4Data g4Data{getGRP4(b)};

                    // Use timestamp from GPS if available.
                    if (1 == g4Data.timeDistance().time1Type()) {
                        sampleTimeStamp = extractTimeDistance(g4Data.timeDistance().time1());
                    }

                    if (nullptr != m_delegateGrp4Data) {
                        m_delegateGrp4Data(g4Data, sampleTimeStamp);
                    }
                }
                else if (POSDecoder::GRP10001 == groupNumber) {
                    // Decode Applanix GRP10001.
                    opendlv::device::gps::pos::Grp10001Data g10001Data{getGRP10001(b, messageSize)};
                    (void)g10001Data;
                }
                else if (POSDecoder::GRP10002 == groupNumber) {
                    // Decode Applanix GRP10002.
                    opendlv::device::gps::pos::Grp10002Data g10002Data{getGRP10002(b, messageSize)};
                    (void)g10002Data;
                }
                else if (POSDecoder::GRP10003 == groupNumber) {
                    // Decode Applanix GRP10003.
                    opendlv::device::gps::pos::Grp10003Data g10003Data{getGRP10003(b)};
                    (void)g10003Data;
                }
                else if (POSDecoder::GRP10009 == groupNumber) {
                    // Decode Applanix GRP10009.
                    opendlv::device::gps::pos::Grp10009Data g10009Data{getGRP10009(b, messageSize)};
                    (void)g10009Data;
                }

                // We have consumed the message, move offset accordingly.
                offset += POSDecoder::GRP_HEADER_SIZE + messageSize;
            }
        }
        else {
            // No $GRP "HEADER bytes found yet; consume one byte and start over.
            offset++;
        }
    }
    // We should not get here as we will leave the state machine inside
    // the while loop at certain point. If we are still getting here,
    // we simply discard everything and start over.
    return size;
}

opendlv::device::gps::pos::TimeDistance POSDecoder::getTimeDistance(std::stringstream &buffer) {
    opendlv::device::gps::pos::TimeDistance timedist;

    if (buffer.good()) {
        double time1            = 0;
        double time2            = 0;
        double distanceTag      = 0;
        uint8_t timeTypes       = 0;
        uint8_t distanceType    = 0;

        buffer.read((char *)(&(time1)), sizeof(time1));
        buffer.read((char *)(&(time2)), sizeof(time2));
        buffer.read((char *)(&(distanceTag)), sizeof(distanceTag));
        buffer.read((char *)(&(timeTypes)), sizeof(timeTypes));
        buffer.read((char *)(&(distanceType)), sizeof(distanceType));

        timedist.time1(time1).time2(time2).distanceTag(distanceTag).time1Type((timeTypes&0x7)).time2Type((timeTypes>>3));
        // TODO: Add enum support to cluon-msc.
//        timedist.setTime1Type(static_cast<opendlv::device::gps::pos::TimeDistance::TimeType>(timeTypes & 0x0F));
//        timedist.setTime2Type(static_cast<opendlv::device::gps::pos::TimeDistance::TimeType>(timeTypes & 0xF0));
//        timedist.setDistanceType(static_cast<opendlv::device::gps::pos::TimeDistance::DistanceType>(distanceType));
    }

    return timedist;
}

opendlv::device::gps::pos::Grp1Data POSDecoder::getGRP1(std::stringstream &buffer) {
    opendlv::device::gps::pos::Grp1Data g1Data;

    if (buffer.good()) {
        // Read timedist field.
        opendlv::device::gps::pos::TimeDistance timedist = getTimeDistance(buffer);

        double lat          = 0;
        double lon          = 0;
        double alt          = 0;
        float vel_north     = 0;
        float vel_east      = 0;
        float vel_down      = 0;
        double roll         = 0;
        double pitch        = 0;
        double heading      = 0;
        double wander       = 0;
        float track         = 0;
        float speed         = 0;
        float arate_lon     = 0;
        float arate_trans   = 0;
        float arate_down    = 0;
        float accel_lon     = 0;
        float accel_trans   = 0;
        float accel_down    = 0;
        uint8_t alignment   = 0;
        uint8_t pad         = 0;

        buffer.read((char *)(&(lat)), sizeof(lat));
        buffer.read((char *)(&(lon)), sizeof(lon));
        buffer.read((char *)(&(alt)), sizeof(alt));
        buffer.read((char *)(&(vel_north)), sizeof(vel_north));
        buffer.read((char *)(&(vel_east)), sizeof(vel_east));
        buffer.read((char *)(&(vel_down)), sizeof(vel_down));
        buffer.read((char *)(&(roll)), sizeof(roll));
        buffer.read((char *)(&(pitch)), sizeof(pitch));
        buffer.read((char *)(&(heading)), sizeof(heading));
        buffer.read((char *)(&(wander)), sizeof(wander));
        buffer.read((char *)(&(track)), sizeof(track));
        buffer.read((char *)(&(speed)), sizeof(speed));
        buffer.read((char *)(&(arate_lon)), sizeof(arate_lon));
        buffer.read((char *)(&(arate_trans)), sizeof(arate_trans));
        buffer.read((char *)(&(arate_down)), sizeof(arate_down));
        buffer.read((char *)(&(accel_lon)), sizeof(accel_lon));
        buffer.read((char *)(&(accel_trans)), sizeof(accel_trans));
        buffer.read((char *)(&(accel_down)), sizeof(accel_down));

        buffer.read((char *)(&(alignment)), sizeof(alignment));
        buffer.read((char *)(&(pad)), sizeof(pad));

        g1Data.lat(lat)
              .lon(lon)
              .alt(alt)
              .vel_north(vel_north)
              .vel_east(vel_east)
              .vel_down(vel_down)
              .roll(roll)
              .pitch(pitch)
              .heading(heading)
              .wander(wander)
              .track(track)
              .speed(speed)
              .arate_lon(arate_lon)
              .arate_trans(arate_trans)
              .arate_down(arate_down)
              .accel_lon(accel_lon)
              .accel_trans(accel_trans)
              .accel_down(accel_down)
              .timeDistance(timedist)
              .alignment(alignment);
    }

    return g1Data;
}

opendlv::device::gps::pos::Grp2Data POSDecoder::getGRP2(std::stringstream &buffer) {
    opendlv::device::gps::pos::Grp2Data g2Data;

    if (buffer.good()) {
        // Read timedist field.
        opendlv::device::gps::pos::TimeDistance timedist = getTimeDistance(buffer);

        float northposrms           = 0;
        float eastposrms            = 0;
        float downposrms            = 0;
        float northvelrms           = 0;
        float eastvelrms            = 0;
        float downvelrms            = 0;
        float rollrms               = 0;
        float pitchrms              = 0;
        float headingrms            = 0;
        float ellipsoidmajor        = 0;
        float ellipsoidminor        = 0;
        float ellipsoidorientation  = 0;
        uint16_t pad                = 0;

        buffer.read((char *)(&(northposrms)), sizeof(northposrms));
        buffer.read((char *)(&(eastposrms)), sizeof(eastposrms));
        buffer.read((char *)(&(downposrms)), sizeof(downposrms));
        buffer.read((char *)(&(northvelrms)), sizeof(northvelrms));
        buffer.read((char *)(&(eastvelrms)), sizeof(eastvelrms));
        buffer.read((char *)(&(downvelrms)), sizeof(downvelrms));
        buffer.read((char *)(&(rollrms)), sizeof(rollrms));
        buffer.read((char *)(&(pitchrms)), sizeof(pitchrms));
        buffer.read((char *)(&(headingrms)), sizeof(headingrms));
        buffer.read((char *)(&(ellipsoidmajor)), sizeof(ellipsoidmajor));
        buffer.read((char *)(&(ellipsoidminor)), sizeof(ellipsoidminor));
        buffer.read((char *)(&(ellipsoidorientation)), sizeof(ellipsoidorientation));

        buffer.read((char *)(&(pad)), sizeof(pad));

        g2Data.northposrms(northposrms)
              .eastposrms(eastposrms)
              .downposrms(downposrms)
              .northvelrms(northvelrms)
              .eastvelrms(eastvelrms)
              .downvelrms(downvelrms)
              .rollrms(rollrms)
              .pitchrms(pitchrms)
              .headingrms(headingrms)
              .ellipsoidmajor(ellipsoidmajor)
              .ellipsoidminor(ellipsoidminor)
              .ellipsoidorientation(ellipsoidorientation)
              .timeDistance(timedist);
    }

    return g2Data;
}

opendlv::device::gps::pos::GNSSReceiverChannelStatus POSDecoder::getGNSSReceiverChannelStatus(std::stringstream &buffer) {
    opendlv::device::gps::pos::GNSSReceiverChannelStatus gnss;

    if (buffer.good()) {
        uint16_t SV_PRN                     = 0;
        uint16_t channel_tracking_status    = 0;
        float SV_azimuth                    = 0;
        float SV_elevation                  = 0;
        float SV_L1_SNR                     = 0;
        float SV_L2_SNR                     = 0;

        buffer.read((char *)(&(SV_PRN)), sizeof(SV_PRN));
        SV_PRN = le16toh(SV_PRN);

        buffer.read((char *)(&(channel_tracking_status)), sizeof(channel_tracking_status));
        channel_tracking_status = le16toh(channel_tracking_status);

        buffer.read((char *)(&(SV_azimuth)), sizeof(SV_azimuth));
        buffer.read((char *)(&(SV_elevation)), sizeof(SV_elevation));
        buffer.read((char *)(&(SV_L1_SNR)), sizeof(SV_L1_SNR));
        buffer.read((char *)(&(SV_L2_SNR)), sizeof(SV_L2_SNR));

        gnss.SV_PRN(SV_PRN)
            .channel_tracking_status(channel_tracking_status)
            .SV_azimuth(SV_azimuth)
            .SV_elevation(SV_elevation)
            .SV_L1_SNR(SV_L1_SNR)
            .SV_L2_SNR(SV_L2_SNR);
    }

    return gnss;
}

opendlv::device::gps::pos::Grp3Data POSDecoder::getGRP3(std::stringstream &buffer) {
    opendlv::device::gps::pos::Grp3Data g3Data;

    if (buffer.good()) {
        // Read timedist field.
        opendlv::device::gps::pos::TimeDistance timedist = getTimeDistance(buffer);

        int8_t navigationSolutionStatus = 0;
        uint8_t numberSVTracked         = 0;
        uint16_t channelStatusByteCount = 0;
        float HDOP                      = 0;
        float VDOP                      = 0;
        float DGPS_correction_latency   = 0;
        uint16_t DGPS_reference_ID      = 0;
        uint32_t UTC_week_number        = 0;
        double UTC_time_offset          = 0;
        float GNSS_navigation_latency   = 0;
        float geoidal_separation        = 0;
        uint16_t GNSS_receiver_type     = 0;
        uint32_t GNSS_status            = 0;
        uint16_t pad                    = 0;

        buffer.read((char *)(&(navigationSolutionStatus)), sizeof(navigationSolutionStatus));
        buffer.read((char *)(&(numberSVTracked)), sizeof(numberSVTracked));
        buffer.read((char *)(&(channelStatusByteCount)), sizeof(channelStatusByteCount));
        channelStatusByteCount = le16toh(channelStatusByteCount);

        const uint8_t SIZE_OF_GNSS{20};
        for (uint8_t i = 0; i < (channelStatusByteCount / SIZE_OF_GNSS); i++) {
            opendlv::device::gps::pos::GNSSReceiverChannelStatus gnss = getGNSSReceiverChannelStatus(buffer);
//            g3Data.addTo_ListOfChannel_status(gnss);
        }

        buffer.read((char *)(&(HDOP)), sizeof(HDOP));
        buffer.read((char *)(&(VDOP)), sizeof(VDOP));
        buffer.read((char *)(&(DGPS_correction_latency)), sizeof(DGPS_correction_latency));
        buffer.read((char *)(&(DGPS_reference_ID)), sizeof(DGPS_reference_ID));
        DGPS_reference_ID = le16toh(DGPS_reference_ID);
        buffer.read((char *)(&(UTC_week_number)), sizeof(UTC_week_number));
        UTC_week_number = le32toh(UTC_week_number);
        buffer.read((char *)(&(UTC_time_offset)), sizeof(UTC_time_offset));
        buffer.read((char *)(&(GNSS_navigation_latency)), sizeof(GNSS_navigation_latency));
        buffer.read((char *)(&(geoidal_separation)), sizeof(geoidal_separation));
        buffer.read((char *)(&(GNSS_receiver_type)), sizeof(GNSS_receiver_type));
        GNSS_receiver_type = le16toh(GNSS_receiver_type);
        buffer.read((char *)(&(GNSS_status)), sizeof(GNSS_status));
        GNSS_status = le32toh(GNSS_status);

        buffer.read((char *)(&(pad)), sizeof(pad));

        // Set values.
        g3Data.navigation_solution_status(navigationSolutionStatus)
              .number_sv_tracked(numberSVTracked)
              .channel_status_byte_count(channelStatusByteCount)
              .HDOP(HDOP)
              .VDOP(VDOP)
              .DGPS_correction_latency(DGPS_correction_latency)
              .DGPS_reference_ID(DGPS_reference_ID)
              .UTC_week_number(UTC_week_number)
              .UTC_time_offset(UTC_time_offset)
              .GNSS_navigation_latency(GNSS_navigation_latency)
              .geoidal_separation(geoidal_separation)
              .GNSS_receiver_type(GNSS_receiver_type)
              .GNSS_status(GNSS_status)
              .timeDistance(timedist);
    }

    return g3Data;
}

opendlv::device::gps::pos::Grp4Data POSDecoder::getGRP4(std::stringstream &buffer) {
    opendlv::device::gps::pos::Grp4Data g4Data;

    if (buffer.good()) {
        // Read timedist field.
        opendlv::device::gps::pos::TimeDistance timedist{getTimeDistance(buffer)};

        constexpr uint16_t LENGTH_IMUDATA = 24;
        char imudata[LENGTH_IMUDATA];
        uint8_t datastatus  = 0;
        uint8_t imutype     = 0;
        uint8_t imurate     = 0;
        uint16_t imustatus  = 0;
        uint8_t pad         = 0;

        buffer.read(imudata, LENGTH_IMUDATA);
        buffer.read((char *)(&(datastatus)), sizeof(datastatus));
        buffer.read((char *)(&(imutype)), sizeof(imutype));
        buffer.read((char *)(&(imurate)), sizeof(imurate));
        buffer.read((char *)(&(imustatus)), sizeof(imustatus));
        imustatus = le16toh(imustatus);
        buffer.read((char *)(&(pad)), sizeof(pad));

        g4Data.imudata(std::string(imudata, LENGTH_IMUDATA))
              .datastatus(datastatus)
              .imutype(imutype)
              .imurate(imurate)
              .imustatus(imustatus)
              .timeDistance(timedist);
    }

    return g4Data;
}

opendlv::device::gps::pos::Grp10001Data POSDecoder::getGRP10001(std::stringstream &buffer, uint32_t payloadSize) {
    opendlv::device::gps::pos::Grp10001Data g10001Data;

    if (buffer.good()) {
        // Read timedist field.
        opendlv::device::gps::pos::TimeDistance timedist{getTimeDistance(buffer)};

        uint16_t GNSS_receiver_type = 0;
        uint32_t reserved           = 0;
        uint16_t byte_count         = 0;
        std::vector<char> GNSS_receiver_raw_data;
        uint8_t pad                 = 0;

        buffer.read((char *)(&(GNSS_receiver_type)), sizeof(GNSS_receiver_type));
        GNSS_receiver_type = le16toh(GNSS_receiver_type);
        buffer.read((char *)(&(reserved)), sizeof(reserved));
        buffer.read((char *)(&(byte_count)), sizeof(byte_count));
        byte_count = le16toh(byte_count);

        // Reserve storage.
        GNSS_receiver_raw_data.reserve(byte_count);
        buffer.read(&GNSS_receiver_raw_data[0], byte_count);

        // Read padding.
        for(uint8_t paddingToRead = 0; paddingToRead < ((payloadSize - TIME_DISTANCE_FIELD_SIZE - sizeof(GNSS_receiver_type) - sizeof(reserved) - sizeof(byte_count) - byte_count - POSDecoder::GRP_FOOTER_SIZE)); paddingToRead++) {
            buffer.read((char *)(&(pad)), sizeof(pad));
        }

        g10001Data.GNSS_receiver_type(GNSS_receiver_type)
                  .GNSS_receiver_raw_data(std::string(&GNSS_receiver_raw_data[0], byte_count))
                  .timeDistance(timedist);
    }

    return g10001Data;
}

opendlv::device::gps::pos::Grp10002Data POSDecoder::getGRP10002(std::stringstream &buffer, uint32_t payloadSize) {
    opendlv::device::gps::pos::Grp10002Data g10002Data;

    if (buffer.good()) {
        // Read timedist field.
        opendlv::device::gps::pos::TimeDistance timedist{getTimeDistance(buffer)};

        constexpr uint16_t LENGTH_IMUHEADER = 6;
        char imuheader[LENGTH_IMUHEADER];
        uint16_t byte_count = 0;
        std::vector<char> imu_raw_data;
        int16_t data_checksum = 0;
        uint8_t pad         = 0;

        buffer.read(imuheader, sizeof(imuheader));
        buffer.read((char *)(&(byte_count)), sizeof(byte_count));
        byte_count = le16toh(byte_count);

        // Reserve storage.
        imu_raw_data.reserve(byte_count);
        buffer.read(&imu_raw_data[0], byte_count);

        buffer.read((char *)(&(data_checksum)), sizeof(data_checksum));
        data_checksum = le16toh(data_checksum);

        // Read padding.
        for(uint8_t paddingToRead = 0; paddingToRead < ((payloadSize - TIME_DISTANCE_FIELD_SIZE - LENGTH_IMUHEADER - sizeof(byte_count) - byte_count - sizeof(data_checksum) - POSDecoder::GRP_FOOTER_SIZE)); paddingToRead++) {
            buffer.read((char *)(&(pad)), sizeof(pad));
        }

        g10002Data.imuheader(std::string(imuheader, LENGTH_IMUHEADER))
                  .imu_raw_data(std::string(&imu_raw_data[0], byte_count))
                  .datachecksum(data_checksum)
                  .timeDistance(timedist);
    }

    return g10002Data;
}

opendlv::device::gps::pos::Grp10003Data POSDecoder::getGRP10003(std::stringstream &buffer) {
    opendlv::device::gps::pos::Grp10003Data g10003Data;

    if (buffer.good()) {
        // Read timedist field.
        opendlv::device::gps::pos::TimeDistance timedist{getTimeDistance(buffer)};

        uint32_t pps = 0;
        buffer.read((char *)(&(pps)), sizeof(pps));
        pps = le32toh(pps);

        uint16_t pad = 0;
        buffer.read((char *)(&(pad)), sizeof(pad));

        g10003Data.pulsecount(pps)
                  .timeDistance(timedist);
    }

    return g10003Data;
}

opendlv::device::gps::pos::Grp10009Data POSDecoder::getGRP10009(std::stringstream &buffer, uint32_t payloadSize) {
    // Grp10009 message is identical to Grp10001. Thus, re-use the decoder and simply copy the data.
    opendlv::device::gps::pos::Grp10001Data g10001Data{getGRP10001(buffer, payloadSize)};

    opendlv::device::gps::pos::Grp10009Data g10009Data;
    g10009Data.GNSS_receiver_type(g10001Data.GNSS_receiver_type())
              .GNSS_receiver_raw_data(g10001Data.GNSS_receiver_raw_data())
              .timeDistance(g10001Data.timeDistance());

    return g10009Data;
}

