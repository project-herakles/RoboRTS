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

#ifndef ROBORTS_SDK_DISPATCH_H
#define ROBORTS_SDK_DISPATCH_H
#include <iostream>       // std::cout
#include <functional>     // std::ref
#include <thread>         // std::thread
#include <future>         // std::promise, std::future
#include <map>
#include <tuple>

#include "handle.h"

namespace roborts_sdk
{
class Handle;

template<typename Cmd>
class SubscriptionCallback
{
public:
    using SharedMessage = typename std::shared_ptr<Cmd>;
    using CallbackType = typename std::function<void(const SharedMessage)>;

    SubscriptionCallback(CallbackType function) :
        callback_function_(function)
    {

    }
    ~SubscriptionCallback() = default;
    void Dispatch(std::shared_ptr<MessageHeader> message_header,
                  SharedMessage message)
    {
        callback_function_(message);
    };
private:
    CallbackType callback_function_;
};

class SubscriptionBase
{
public:
    SubscriptionBase(std::shared_ptr<Handle> handle, uint16_t cmd_id) :
        handle_(handle)
    {
        cmd_info_ = std::make_shared<CommandInfo>();
        cmd_info_->cmd_id = cmd_id;
    }
    ~SubscriptionBase() = default;
    std::shared_ptr<Handle> GetHandle()
    {
        return handle_;
    }
    std::shared_ptr<CommandInfo> GetCommandInfo()
    {
        return cmd_info_;
    }
    std::shared_ptr<MessageHeader> CreateMessageHeader()
    {
        return std::shared_ptr<MessageHeader>(new MessageHeader);
    }

    virtual std::shared_ptr<void> CreateMessage() = 0;
    virtual void HandleMessage(std::shared_ptr<MessageHeader> message_header, std::shared_ptr<void> message) = 0;
protected:
    std::shared_ptr<Handle> handle_;
    std::shared_ptr<CommandInfo> cmd_info_;

};

template<typename Cmd>
class Subscription : public SubscriptionBase
{
public:
    using SharedMessage = typename std::shared_ptr<Cmd>;
    using CallbackType = typename std::function<void(const SharedMessage)>;

    Subscription(std::shared_ptr<Handle> handle, uint16_t cmd_id,
                 CallbackType &&function) :
        SubscriptionBase(handle, cmd_id),
        callback_(std::forward<CallbackType>(function))
    {
        cmd_info_->length = sizeof(Cmd);
    }
    ~Subscription() = default;
    std::shared_ptr<void> CreateMessage()
    {
        return std::shared_ptr<void>(new Cmd);
    }
    void HandleMessage(std::shared_ptr<MessageHeader> message_header, std::shared_ptr<void> message)
    {
        auto typed_message = std::static_pointer_cast<Cmd>(message);
        callback_.Dispatch(message_header, typed_message);
    }
private:
    SubscriptionCallback<Cmd> callback_;
};

class PublisherBase
{
public:
    PublisherBase(std::shared_ptr<Handle> handle, uint16_t cmd_id) :
        handle_(handle)
    {
        cmd_info_ = std::make_shared<CommandInfo>();
        cmd_info_->cmd_id = cmd_id;
    }
    ~PublisherBase() = default;
    std::shared_ptr<Handle> GetHandle()
    {
        return handle_;
    }
    std::shared_ptr<CommandInfo> GetCommandInfo()
    {
        return cmd_info_;
    }

protected:
    std::shared_ptr<Handle> handle_;
    std::shared_ptr<CommandInfo> cmd_info_;
};

template<typename Cmd>
class Publisher : public PublisherBase
{
public:
    Publisher(std::shared_ptr<Handle> handle, uint16_t cmd_id) :
        PublisherBase(handle, cmd_id)
    {
        cmd_info_->length = sizeof(Cmd);
    }
    ~Publisher() = default;

    void Publish(Cmd &message)
    {
        bool ret = GetHandle()->GetProtocol()->Send(GetCommandInfo().get(), &message);
        if (!ret)
        {
            DLOG_ERROR << "send message failed!";
        }
    }
};
}
#endif //ROBORTS_SDK_DISPATCH_H
