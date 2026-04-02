#include "hal/LinuxI2CDevice.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>

namespace solar {

LinuxI2CDevice::~LinuxI2CDevice() {
    close();
}

bool LinuxI2CDevice::open(const I2CDeviceSettings& settings) {
    close();

    char devpath[32];
    std::snprintf(devpath, sizeof(devpath), "/dev/i2c-%d", settings.bus);

    fd_ = ::open(devpath, O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        fd_ = -1;
        return false;
    }

    // Bind the fd to one slave address so later reads and writes stay simple.
    if (::ioctl(fd_, I2C_SLAVE, static_cast<int>(settings.address)) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    settings_ = settings;
    return true;
}

void LinuxI2CDevice::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool LinuxI2CDevice::isOpen() const {
    return fd_ >= 0;
}

bool LinuxI2CDevice::writeByte(const std::uint8_t reg, const std::uint8_t value) {
    if (fd_ < 0) {
        return false;
    }

    const std::uint8_t buffer[2] = {reg, value};
    return ::write(fd_, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

bool LinuxI2CDevice::readByte(const std::uint8_t reg, std::uint8_t& value) {
    if (fd_ < 0) {
        return false;
    }

    if (::write(fd_, &reg, 1) != 1) {
        return false;
    }

    return ::read(fd_, &value, 1) == 1;
}

bool LinuxI2CDevice::readBytes(const std::uint8_t reg, std::uint8_t* dest, const std::size_t count) {
    if (fd_ < 0 || dest == nullptr || count == 0U) {
        return false;
    }

    if (::write(fd_, &reg, 1) != 1) {
        return false;
    }

    return ::read(fd_, dest, count) == static_cast<ssize_t>(count);
}

} // namespace solar