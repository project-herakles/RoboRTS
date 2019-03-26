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

#include "protocol.h"
#include "protocol_define.h"
#include <iomanip>

namespace roborts_sdk
{

Protocol::Protocol(std::shared_ptr<SerialDevice> serial_device_ptr) :
    running_(false),
    serial_device_ptr_(serial_device_ptr), seq_num_(0)
{

}

Protocol::~Protocol()
{
    running_ = false;
}

bool Protocol::Init()
{

    seq_num_ = 0;
    auto max_buffer_size = BUFFER_SIZE;
    auto max_pack_size = MAX_PACK_SIZE;
    auto session_table_num = SESSION_TABLE_NUM;
    memory_pool_ptr_ = std::make_shared<MemoryPool>(max_buffer_size,
                       max_pack_size,
                       session_table_num);
    memory_pool_ptr_->Init();
    SetupSession();
    running_ = true;
    return true;
}

/*************************** Session Management **************************/
void Protocol::SetupSession()
{
    cmd_session.session_id = 0;
    cmd_session.usage_flag = false;
    cmd_session.memory_block_ptr = nullptr;
}

/****************************** Send Pipline *****************************/
bool Protocol::Send(const CommandInfo *command_info, void *data_ptr)
{
    Header *header_ptr = nullptr;
    uint16_t crc_data, pack_length = 0;
    const uint16_t cmd_id = command_info->cmd_id, data_length = command_info->length;

    //calculate pack_length first
    if (data_length == 0 || data_ptr == nullptr)
    {
        DLOG_ERROR << "No data send.";
        return false;
    }
    // As icra2018 protocol: Header -> cmd_id -> payload_len -> payload_crc16
    pack_length = HEADER_LEN + CMD_ID_LEN + data_length + CRC_DATA_LEN;

    //lock
    memory_pool_ptr_->LockMemory();

    //alloc session
    if (cmd_session.usage_flag)
    {
        memory_pool_ptr_->UnlockMemory();
        DLOG_ERROR << "TX occupied";
        return false;
    }

    if((cmd_session.memory_block_ptr = memory_pool_ptr_->AllocMemory(pack_length)) == nullptr)
    {
        memory_pool_ptr_->UnlockMemory();
        DLOG_ERROR << "TX memory allocation failed";
        return false;
    }
    cmd_session.usage_flag = true;

    //pack into cmd_session memory_block
    header_ptr = (Header *) cmd_session.memory_block_ptr->memory_ptr;
    header_ptr->sof = SOF;
    header_ptr->data_length = data_length;
    header_ptr->seq = seq_num_;
    header_ptr->crc8 = CRC8Calc(cmd_session.memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

    // pack the cmd prefix ,data and data crc into memory block one by one
    memcpy(cmd_session.memory_block_ptr->memory_ptr + HEADER_LEN, &cmd_id, CMD_ID_LEN);
    memcpy(cmd_session.memory_block_ptr->memory_ptr + HEADER_LEN + CMD_ID_LEN, data_ptr, data_length);

    crc_data = CRC16Calc(cmd_session.memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
    memcpy(cmd_session.memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);

    // send it using device
    DeviceSend(cmd_session.memory_block_ptr->memory_ptr);

    seq_num_++;

    memory_pool_ptr_->FreeMemory(cmd_session.memory_block_ptr);
    cmd_session.usage_flag = false;

    //unlock
    memory_pool_ptr_->UnlockMemory();
    return true;
}

bool Protocol::DeviceSend(uint8_t *buf)
{
    int ans;
    Header *header_ptr = (Header *) buf;

// For debug and visualzation:
// ans = header_ptr->length;
    printf("Transmitting [ ");
    for(int i = 0; i < header_ptr->data_length; i++)
        printf("%02x ", buf[i]);
    printf("]\n");
    ans = serial_device_ptr_->Write(buf, header_ptr->data_length);

    if (ans <= 0)
    {
        DLOG_ERROR << "Port failed.";
    }
    else if (ans != header_ptr->data_length)
    {
        DLOG_ERROR << "Port send failed, send length:" << ans << "package length" << header_ptr->data_length;
    }
    else
    {
        DLOG_INFO << "Port send success.";
        return true;
    }
    return false;
}


}
