#include "LidarDriver.h"

namespace lidar {

Driver::Driver(HardwareSerial& serial)
    : serial_(serial),
	  speedController_(2),
      //frameAvailable_(false),
      started_(false)
{
}

void Driver::begin()
{
    serial_.begin(230400);

    parser_.reset();

    //frameAvailable_ = false;

	frameQueue_.clear();

    started_ = true;

	speedController_.begin();
	speedController_.setDutyCycle(0.20f);
}

void Driver::stop()
{
    serial_.end();

    started_ = false;

    //frameAvailable_ = false;
	frameQueue_.clear();

    parser_.reset();
}

// -----------------------------------------------------------------------
// Process — lecture série et alimentation de la deque
// -----------------------------------------------------------------------

void Driver::process()
{
    if (!started_) {
        return;
    }

    while (serial_.available() > 0) {
        const uint8_t byte =
            static_cast<uint8_t>(serial_.read());

        if (parser_.pushByte(byte, rxFrame_)) {

            // // Une trame complète et valide vient d'être reçue.
            // memcpy(
            //     readyFrame_,
            //     rxFrame_,
            //     ProtocolParser::FRAME_LENGTH
            // );

            // frameAvailable_ = true;

			// debugCounter++;

            // Une trame complète et valide vient d'être reçue.
            // On la copie dans un std::array pour l'insérer dans la deque.
            RawFrame newFrame;
            memcpy(newFrame.data(), rxFrame_, ProtocolParser::FRAME_LENGTH);
 
            // Si la deque est pleine, on supprime la trame la plus ancienne
            // (back) pour faire de la place à la nouvelle (front).
            if (frameQueue_.size() >= MAX_FRAME_QUEUE_SIZE) {
                frameQueue_.pop_back();
            }
 
            // Insertion en front : la trame la plus récente est toujours
            // accessible via front().
            frameQueue_.push_front(newFrame);
        }
    }
}

// -----------------------------------------------------------------------
// GetData — retourne la trame la plus récente
// -----------------------------------------------------------------------

bool Driver::GetLastFrame(Data& data)
{
    //if (!frameAvailable_) {
	if (frameQueue_.empty()) {
        return false;
    }

    // frameAvailable_ = false;
	// Serial.println(debugCounter);
	// debugCounter = 0;
    // return dataConverter_.convert(readyFrame_, data);

    // Récupère et consomme la trame la plus récente (front).
    const RawFrame& frame = frameQueue_.front();
    const bool ok = dataConverter_.convert(frame.data(), data);
    frameQueue_.pop_front();

	//Serial.println(frameCount());

	frameQueue_.clear();
 
    return ok;
}

// -----------------------------------------------------------------------
// Accesseur
// -----------------------------------------------------------------------
 
size_t Driver::frameCount() const
{
    return frameQueue_.size();
}

} // namespace lidar