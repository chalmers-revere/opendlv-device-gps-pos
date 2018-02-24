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
#include <array>
#include <sstream>
#include <string>

POSDecoder::POSDecoder(std::function<void(const double &latitude, const double &longitude, const std::chrono::system_clock::time_point &tp)> delegateLatitudeLongitude,
                         std::function<void(const float &heading, const std::chrono::system_clock::time_point &tp)> delegateHeading) noexcept
    : m_delegateLatitudeLongitude(std::move(delegateLatitudeLongitude))
    , m_delegateHeading(std::move(delegateHeading)) {}

void POSDecoder::decode(const std::string &data, std::chrono::system_clock::time_point &&tp) noexcept {
    const std::chrono::system_clock::time_point timestamp{tp};
    (void)timestamp;
    (void)data;
}

