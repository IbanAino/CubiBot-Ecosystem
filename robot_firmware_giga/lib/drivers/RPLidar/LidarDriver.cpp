#include "LidarDriver.h"

namespace lidar {

Driver::Driver(HardwareSerial& serial)
    : serial_(serial),
	  speedController_(2),
      started_(false)
{
}

void Driver::begin()
{
    serial_.begin(230400);
    parser_.reset();
    started_ = true;
	speedController_.begin();
	speedController_.setDutyCycle(0.20f);
}

void Driver::stop()
{
    serial_.end();
    started_ = false;
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
			if(parser_.calculateCRC8(rxFrame_, 47) == 0){

				//debugCounter1++;
				//Serial.print("Trames entrantes : ");
				//Serial.println(debugCounter1);

				// uint16_t timestamp =
				// 	static_cast<uint16_t>(rxFrame_[44]) |
				// 	(static_cast<uint16_t>(rxFrame_[45]) << 8
				// );

				// Serial.print(timestamp);
				// // Serial.println(rxFrame_[0]);
				// // Serial.println(rxFrame_[1]);

				// Serial.print("   -   ");

				// Serial.println(parser_.calculateCRC8(rxFrame_, 47));

				// Une trame complète et valide vient d'être reçue.
				// On la copie dans un std::array pour l'insérer dans la deque.
				RawFrame newFrame;
				memcpy(newFrame.data(), rxFrame_, ProtocolParser::FRAME_LENGTH);

				// Si la deque est pleine, on supprime la trame la plus ancienne (au front)
				if (frameQueue_.size() >= MAX_FRAME_QUEUE_SIZE) {
				    frameQueue_.pop_front();
					//debugCounter++;
				}

				//Serial.println(debugCounter);
	
				// On insère la nouvelle trame à l'arrière (back)
				frameQueue_.push_back(newFrame);
			}
        }
    }
}

// -----------------------------------------------------------------------
// GetData — retourne la trame la plus récente
// -----------------------------------------------------------------------

// bool Driver::GetLastFrame(Data& data)
// {
//     //if (!frameAvailable_) {
// 	if (frameQueue_.empty()) {
//         return false;
//     }

//     // frameAvailable_ = false;
// 	// Serial.println(debugCounter);
// 	// debugCounter = 0;
//     // return dataConverter_.convert(readyFrame_, data);

//     // Récupère et consomme la trame la plus récente (front).
//     const RawFrame& frame = frameQueue_.front();
//     const bool ok = dataConverter_.convert(frame.data(), data);
//     frameQueue_.pop_front();

// 	//Serial.println(frameCount());
// 	//Serial.println(data.startAngle);

// 	//frameQueue_.clear();
 
//     return ok;
// }

/**
 * @brief Extrait jusqu'à maxCount trames de la deque dans batch.
 *
 * @param batch     Tableau destination (alloué par l'appelant).
 * @param maxCount  Taille maximale du tableau batch.
 * @return          Nombre de trames effectivement copiées (0 si deque vide).
 */
uint8_t Driver::GetFrames(
    lidar::Data* batch,
    uint8_t maxCount)
{
	if (frameQueue_.empty()) {
        return 0;
    }
	
	uint8_t count = 0;
    while (!frameQueue_.empty() && count < maxCount) {
        // Conversion directe depuis la deque vers le batch
        dataConverter_.convert(
			//frameQueue_.front().data(),
			frameQueue_.front().data(), // récupère l'élément le plus ANCIEN (à l'arrière de la deque)
			batch[count]
		);

		frameQueue_.pop_front();  // supprime l'élément le plus ancien
        count++;
    }

	// debugCounter2 += count;
	// Serial.print("Trames sortantes : ");
	// Serial.println(debugCounter2);

    return count;
}

// -----------------------------------------------------------------------
// Accesseur
// -----------------------------------------------------------------------
 
size_t Driver::frameCount() const
{
    return frameQueue_.size();
}

} // namespace lidar