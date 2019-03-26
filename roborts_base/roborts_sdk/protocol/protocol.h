/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_SDK_PROTOCOL_H
#define ROBORTS_SDK_PROTOCOL_H
#include <memory>
#include <cstring>

#include "../hardware/serial_device.h"
#include "../utilities/memory_pool.h"
#include "../utilities/circular_buffer.h"
#include "../utilities/crc.h"
#include <map>
#include <atomic>
#include <thread>

namespace roborts_sdk
{
/*************************** Package Format **************************/
/**
 * @brief Package header used to resolve package
 */
#pragma pack(push, 1)
typedef struct Header
{
    uint8_t sof : 8;
    uint16_t data_length : 16;
    uint8_t seq : 8;
    uint8_t crc8 : 8;
} Header;
#pragma pack(pop)

/*************************** Package Infomation **********************/
/**
 * @brief Key information that a protocol command should include,
 * @details Used as an interface with dispatch layer.
 */
typedef struct CommandInfo
{
    uint16_t cmd_id;
    uint16_t length;
} CommandInfo;

/**
 * @brief Message header
 * @details Used as an interface with dispatch layer.
 */
typedef struct MessageHeader
{
    uint16_t seq_num;
    uint8_t session_id;
    bool is_ack;
} MessageHeader;

/**
 * @brief Message data
 * @details Used as an interface with dispatch layer.
 */
typedef union MessageData
{
    uint8_t raw_data[1024];
} MessageData;

/************************* Session Information ***********************/

/**
 * @brief Information for command session
 */
typedef struct CMDSession
{
    //! session id used to distinguish the session mode
    uint8_t session_id;
    //! whether the cmd session is used
    bool usage_flag;
    //! memory block which holds the package for the command session
    MemoryBlock *memory_block_ptr;
} CMDSession;


/**
 * @brief Class for protocol layer.
 */
class Protocol
{
public:
    /**
     * @brief Constructor for protocol
     * @param serial_device_ptr Pointer for serial device
     */
    explicit Protocol(std::shared_ptr<SerialDevice> serial_device_ptr);
    /**
     * @brief Destructor for protocol
     */
    ~Protocol();
    /***************************** Interface ****************************/
    /**
     * @brief Initialize memory pool, stream, container and session,
     *        start the automatic repeat sending thread and receiving pool thread
     * @return True if success
     */
    bool Init();
    /*************************** Send Pipline ***************************/
    /**
    * @brief An interface function for dispatch layer to send cmd without need for ack in the protocol layer
    * @param command_info Input command information
    * @param message_data Input message data
    * @return True if command is successfully allocated and sent by protocol layer
    */
    bool Send(const CommandInfo *command_info, void *message_data);
    /**
     * @brief Use hardware interface in the hardware layer to send the data
     * @param buf pointer for the buffer head
     * @return True if the buffer is successfully sent, blocked to retry connection
     *         if the hardware device is disconnected after connection.
     */
    bool DeviceSend(uint8_t *buf, const uint16_t pack_length);

    /************************ Session Management ***********************/
    /**
     * @brief Setup the command and ack session for initialization
     */
    void SetupSession();

    /******************* Const List ***************************/

    //! size of receive buffer used to read from hardware device
    static const size_t BUFFER_SIZE = 1024;
    //! max Size of package
    static const size_t MAX_PACK_SIZE = 1024;
    //! session number for a sender/receiver
    static const size_t SESSION_TABLE_NUM = 32;
    //! length of header
    static const size_t HEADER_LEN = sizeof(Header);
    //! length of CRC8
    static const size_t CRC_HEAD_LEN = sizeof(uint8_t);
    //! length of CRC16
    static const size_t CRC_DATA_LEN = sizeof(uint16_t);
    //! length of CMD_ID as in icra2018
    static const size_t CMD_ID_LEN = sizeof(uint16_t);

    //! SOF
    static const uint8_t SOF = 0xA0;

private:
    //! shared pointer of serial device
    std::shared_ptr<SerialDevice> serial_device_ptr_;
    //! shared pointer of memory pool
    std::shared_ptr<MemoryPool> memory_pool_ptr_;

    //! sequence number
    uint16_t seq_num_;

    //! command session table
    CMDSession cmd_session;

    //! if receive pool should run
    std::atomic<bool> running_;
};
}
#endif //ROBORTS_SDK_PROTOCOL_H
