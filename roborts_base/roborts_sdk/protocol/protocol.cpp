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
    serial_device_ptr_(serial_device_ptr), seq_num_(0),
    is_large_data_protocol_(true), reuse_buffer_(true),
    poll_tick_(10)
{

}

Protocol::~Protocol()
{
    running_ = false;
    if (send_poll_thread_.joinable())
    {
        send_poll_thread_.join();
    }
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
    send_poll_thread_ = std::thread(&Protocol::AutoRepeatSendCheck, this);
    return true;
}

void Protocol::AutoRepeatSendCheck()
{
    while (running_)
    {
        unsigned int i;

        std::chrono::steady_clock::time_point current_time_stamp;

        for (i = 1; i < SESSION_TABLE_NUM; i++)
        {

            if (cmd_session_table_[i].usage_flag == 1)
            {
                current_time_stamp = std::chrono::steady_clock::now();
                if ((std::chrono::duration_cast<std::chrono::milliseconds>
                        (current_time_stamp - cmd_session_table_[i].pre_time_stamp) >
                        cmd_session_table_[i].ack_timeout))
                {

                    memory_pool_ptr_->LockMemory();
                    if (cmd_session_table_[i].retry_time > 0)
                    {

                        if (cmd_session_table_[i].sent >= cmd_session_table_[i].retry_time)
                        {
                            LOG_ERROR << "Sending timeout, Free session "
                                      << static_cast<int>(cmd_session_table_[i].session_id);
                            FreeCMDSession(&cmd_session_table_[i]);
                        }
                        else
                        {
                            LOG_ERROR << "Retry session "
                                      << static_cast<int>(cmd_session_table_[i].session_id);
                            DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr);
                            cmd_session_table_[i].pre_time_stamp = current_time_stamp;
                            cmd_session_table_[i].sent++;
                        }
                    }
                    else
                    {
                        DLOG_ERROR << "Send once " << i;
                        DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr);
                        cmd_session_table_[i].pre_time_stamp = current_time_stamp;
                    }
                    memory_pool_ptr_->UnlockMemory();
                }
                else
                {
//        DLOG_INFO<<"Wait for timeout Session: "<< i;
                }
            }
        }
        usleep(1000);
    }
}

bool Protocol::SendMessage(const CommandInfo *command_info,
                           void *message_data)
{
    return SendCMD(command_info->cmd_set, command_info->cmd_id,
                   command_info->receiver, message_data, command_info->length,
                   CMDSessionMode::CMD_SESSION_0);
}

/*************************** Session Management **************************/
void Protocol::SetupSession()
{
    uint8_t i, j;
    for (i = 0; i < SESSION_TABLE_NUM; i++)
    {
        cmd_session_table_[i].session_id = i;
        cmd_session_table_[i].usage_flag = false;
        cmd_session_table_[i].memory_block_ptr = nullptr;
    }

    for (i = 0; i < RECEIVER_NUM; i++)
    {
        for (j = 0; j < (SESSION_TABLE_NUM - 1); j++)
        {
            ack_session_table_[i][j].session_id = j + 1;
            ack_session_table_[i][j].session_status = ACKSessionStatus::ACK_SESSION_IDLE;
            ack_session_table_[i][j].memory_block_ptr = nullptr;
        }
    }
}

CMDSession *Protocol::AllocCMDSession(CMDSessionMode session_mode, uint16_t size)
{
    uint32_t i;
    MemoryBlock *memory_block_ptr = nullptr;

    if (session_mode == CMDSessionMode::CMD_SESSION_0 || session_mode == CMDSessionMode::CMD_SESSION_1)
    {
        if (cmd_session_table_[(uint16_t) session_mode].usage_flag == 0)
        {
            i = static_cast<uint32_t>(session_mode);
        }
        else
        {
            DLOG_ERROR << "session " << static_cast<uint32_t>(session_mode) << " is busy\n";
            return nullptr;
        }
    }
    else
    {
        for (i = 2; i < SESSION_TABLE_NUM; i++)
        {
            if (cmd_session_table_[i].usage_flag == 0)
            {
                break;
            }
        }

    }

    if (i < 32 && cmd_session_table_[i].usage_flag == 0)
    {

        cmd_session_table_[i].usage_flag = 1;
        memory_block_ptr = memory_pool_ptr_->AllocMemory(size);
        if (memory_block_ptr == nullptr)
        {
            cmd_session_table_[i].usage_flag = 0;
        }
        else
        {
//      DLOG_INFO<<"find "<<i;
            cmd_session_table_[i].memory_block_ptr = memory_block_ptr;
            return &cmd_session_table_[i];
        }
    }
    else
    {
        DLOG_INFO << "All usable CMD session id are occupied";
    }

    return nullptr;
}

void Protocol::FreeCMDSession(CMDSession *session_ptr)
{
    if (session_ptr->usage_flag == 1)
    {
        memory_pool_ptr_->FreeMemory(session_ptr->memory_block_ptr);
        session_ptr->usage_flag = 0;
    }
}


/****************************** Send Pipline *****************************/
bool Protocol::SendCMD(uint8_t cmd_set, uint8_t cmd_id, uint8_t receiver,
                       void *data_ptr, uint16_t data_length,
                       CMDSessionMode session_mode, MessageHeader* message_header,
                       std::chrono::milliseconds ack_timeout, int retry_time)
{
    if(cmd_set != CMD_SET_GIMBAL_ANGLE) return false;

    CMDSession *cmd_session_ptr = nullptr;
    Header *header_ptr = nullptr;
    uint8_t cmd_set_prefix[] = {cmd_id, cmd_set};
    uint32_t crc_data;

    uint16_t pack_length = 0;


    //calculate pack_length first
    if (data_length == 0 || data_ptr == nullptr)
    {
        DLOG_ERROR << "No data send.";
        return false;
    }
    pack_length = HEADER_LEN +
                  CMD_SET_PREFIX_LEN +
                  data_length + CRC_DATA_LEN;

    //second get the param into the session
    switch (session_mode)
    {

    case CMDSessionMode::CMD_SESSION_0:
        //lock
        memory_pool_ptr_->LockMemory();
        cmd_session_ptr = AllocCMDSession(CMDSessionMode::CMD_SESSION_0, pack_length);

        if (cmd_session_ptr == nullptr)
        {
            //unlock
            memory_pool_ptr_->UnlockMemory();
            DLOG_ERROR << "Allocate CMD session failed.";
            return false;
        }

        //pack into cmd_session memory_block
        header_ptr = (Header *) cmd_session_ptr->memory_block_ptr->memory_ptr;
        header_ptr->sof = SOF;
        header_ptr->data_length = pack_length;
        header_ptr->seq = seq_num_;
        header_ptr->crc8 = CRC8Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

        if(message_header)
        {
            message_header->is_ack = false;
            message_header->seq_num = seq_num_;
            message_header->session_id = cmd_session_ptr->session_id;
        }

        // pack the cmd prefix ,data and data crc into memory block one by one
        memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, cmd_set_prefix, CMD_SET_PREFIX_LEN);
        memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN, data_ptr, data_length);

        crc_data = CRC32Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
        memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);

        // send it using device
        DeviceSend(cmd_session_ptr->memory_block_ptr->memory_ptr);

        seq_num_++;
        FreeCMDSession(cmd_session_ptr);
        //unlock
        memory_pool_ptr_->UnlockMemory();
        break;
    default:
        //DLOG_ERROR << "session mode is not valid";
        return false;
    }

    return true;

}

bool Protocol::DeviceSend(uint8_t *buf)
{
    int ans;
    Header *header_ptr = (Header *) buf;

// For debug and visualzation:
// ans = header_ptr->length;
//  for(int i =0;i<header_ptr->length;i++){
//    printf("send_byte %d:\t %X\n ", i, buf[i]);
//  }
//  std::cout<<"----------------"<<std::endl;
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


/*************************** CRC Calculationns ****************************/
uint16_t Protocol::CRC16Update(uint16_t crc, uint8_t ch)
{
    uint16_t tmp;
    uint16_t msg;

    msg = 0x00ff & static_cast<uint16_t>(ch);
    tmp = crc ^ msg;
    crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

    return crc;
}

uint32_t Protocol::CRC32Update(uint32_t crc, uint8_t ch)
{
    uint32_t tmp;
    uint32_t msg;

    msg = 0x000000ffL & static_cast<uint32_t>(ch);
    tmp = crc ^ msg;
    crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
    return crc;
}

uint16_t Protocol::CRC16Calc(const uint8_t *data_ptr, size_t length)
{
    size_t i;
    uint16_t crc = CRC_INIT;

    for (i = 0; i < length; i++)
    {
        crc = CRC16Update(crc, data_ptr[i]);
    }

    return crc;
}

uint32_t Protocol::CRC32Calc(const uint8_t *data_ptr, size_t length)
{
    size_t i;
    uint32_t crc = CRC_INIT;

    for (i = 0; i < length; i++)
    {
        crc = CRC32Update(crc, data_ptr[i]);
    }

    return crc;
}

bool Protocol::CRCHeadCheck(uint8_t *data_ptr, size_t length)
{
    if (CRC16Calc(data_ptr, length) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Protocol::CRCTailCheck(uint8_t *data_ptr, size_t length)
{
    if (CRC32Calc(data_ptr, length) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
}
