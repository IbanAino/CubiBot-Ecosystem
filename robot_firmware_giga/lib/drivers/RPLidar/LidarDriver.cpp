#include "LidarDriver.h"

namespace lidar {

Driver::Driver(HardwareSerial& serial)
    : serial_(serial),
	  speedController_(2),
      frameAvailable_(false),
      started_(false)
{
}

void Driver::begin()
{
    serial_.begin(230400);

    parser_.reset();

    frameAvailable_ = false;
    started_ = true;

	speedController_.begin();
	speedController_.setDutyCycle(0.20f);
}

void Driver::stop()
{
    serial_.end();

    started_ = false;
    frameAvailable_ = false;

    parser_.reset();
}

void Driver::process()
{
    if (!started_) {
        return;
    }

    while (serial_.available() > 0) {
        const uint8_t byte =
            static_cast<uint8_t>(serial_.read());

        if (parser_.pushByte(byte, rxFrame_)) {

            // Une trame complète et valide vient d'être reçue.
            memcpy(
                readyFrame_,
                rxFrame_,
                ProtocolParser::FRAME_LENGTH
            );

            frameAvailable_ = true;
        }
    }
}

bool Driver::GetData(Data& data)
{
    if (!frameAvailable_) {
        return false;
    }

    frameAvailable_ = false;

    return dataConverter_.convert(readyFrame_, data);
}

} // namespace lidar