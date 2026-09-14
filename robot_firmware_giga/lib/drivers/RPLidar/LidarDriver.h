// PacketLidar = 88 octets

// ┌──────────────┬──────────────┬──────────────────────────────────────┐
// │ timestamp    │ speed        │ 12 points                            │
// │ uint16_t     │ uint16_t     │                                      │
// │ 2 octets     │ 2 octets     │ 12 × 7 octets = 84 octets           │
// └──────────────┴──────────────┴──────────────────────────────────────┘
//        0              2                       4                  88




#pragma once

#include <Arduino.h>
#include <array>
#include <deque>
#include <stdint.h>
#include <rtos.h>

#include "LidarTypes.h"
#include "LidarProtocol.h"
#include "LidarDataConverter.h"
#include "LidarSpeedController.h"

namespace lidar {

class Driver
{
public:


    Driver(HardwareSerial& serial);

    void begin();
    void stop();
    void process();

    //bool GetLastFrame(Data& data);

	uint8_t GetFrames(
		lidar::Data* batch,
		uint8_t maxCount)
	;

	/** @brief Retourne le nombre de trames actuellement dans la deque. */
    size_t frameCount() const;

private:
    HardwareSerial& serial_;
	SpeedController speedController_;

    ProtocolParser parser_;
    DataConverter dataConverter_;

    uint8_t rxFrame_[ProtocolParser::FRAME_LENGTH];
    //uint8_t readyFrame_[ProtocolParser::FRAME_LENGTH];

    //bool frameAvailable_;
    bool started_;

	//int debugCounter = 0;
   	// Chaque élément de la deque est une trame brute (tableau d'octets).
    // std::array évite d'utiliser un pointeur brut et garantit la copie
    // correcte lors de l'insertion dans la deque.
    using RawFrame = std::array<uint8_t, ProtocolParser::FRAME_LENGTH>;

    /**
     * @brief Deque FILO de trames brutes.
     *
     * Convention :
     *   front() → trame la plus RÉCENTE  (insérée en dernier)
     *   back()  → trame la plus ANCIENNE (insérée en premier)
     *
     * Insertion : push_front() après chaque trame valide reçue.
     * Éviction   : pop_back()  si la deque est pleine avant insertion.
     * Consommation : pop_front() dans GetData().
     */
	//mutable rtos::Mutex queueMutex_;
    std::deque<RawFrame> frameQueue_;
};

} // namespace lidar