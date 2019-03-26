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

#ifndef ROBORTS_SDK_HANDLE_H
#define ROBORTS_SDK_HANDLE_H
#include "../protocol/protocol.h"
#include "dispatch.h"

namespace roborts_sdk
{
class SubscriptionBase;
class PublisherBase;

template<typename Cmd>
class Subscription;
template<typename Cmd>
class Publisher;
/**
 * @brief Handle class in the dispatch layer
 */
class Handle : public std::enable_shared_from_this<Handle>
{
    //TODO: make this singleton
public:
    template<typename Cmd>
    friend
    class Subscription;
    template<typename Cmd>
    friend
    class Publisher;
    /**
     * @brief Constructor of Handle, instantiate the object of the hardware layer and protocol layer
     * @param serial_port
     */
    explicit Handle(std::string serial_port);
    /**
     * @brief Initialize the hardware layer and protocol layer
     * @return True if both initialize successfully;
     */
    bool Init();
    /**
     * @brief Get the pointer of protocol layer
     * @return The pointer of protocol layer
     */
    std::shared_ptr<Protocol> GetProtocol();
    /**
     * @brief Create the subscriber for the protocol command without need of ack (Receive command)
     * @tparam Cmd Command DataType
     * @param cmd_set Command set for different module, i.e. gimbal, chassis
     * @param cmd_id Command id for different commands in the module
     * @param sender Sender address
     * @param receiver Receiver address
     * @param function Subscriber Callback function
     * @return Pointer of subscription handle
     */
    template<typename Cmd>
    std::shared_ptr<Subscription<Cmd>> CreateSubscriber(uint8_t cmd_set, uint8_t cmd_id,
                                    uint8_t sender, uint8_t receiver,
                                    typename Subscription<Cmd>::CallbackType &&function)
    {
        auto subscriber = std::make_shared<Subscription<Cmd>>(shared_from_this(),
                          cmd_set, cmd_id,
                          sender, receiver,
                          std::forward<typename Subscription<Cmd>::CallbackType>(
                              function));
        subscription_factory_.push_back(
            std::dynamic_pointer_cast<SubscriptionBase>(subscriber));
        return subscriber;
    }
    /**
     * @brief Create the publisher for the protocol command without need of ack (Send command)
     * @tparam Cmd Command DataType
     * @param cmd_set Command set for different module, i.e. gimbal, chassis
     * @param cmd_id Command id for different commands in the module
     * @param sender Sender address
     * @param receiver Receiver address
     * @return Pointer of publisher handle
     */
    template<typename Cmd>
    std::shared_ptr<Publisher<Cmd>> CreatePublisher(uint8_t cmd_set, uint8_t cmd_id, uint8_t sender, uint8_t receiver)
    {
        auto publisher = std::make_shared<Publisher<Cmd>>(shared_from_this(),
                         cmd_set, cmd_id,
                         sender, receiver);
        publisher_factory_.push_back(
            std::dynamic_pointer_cast<PublisherBase>(publisher));
        return publisher;
    }
private:
    //! vector of subsctription base pointers
    std::vector<std::shared_ptr<SubscriptionBase>> subscription_factory_;
    //! vector of publisher base pointers
    std::vector<std::shared_ptr<PublisherBase>> publisher_factory_;

    //! pointer of hardware layer
    std::shared_ptr<SerialDevice> device_;
    //! pointer of protocol layer
    std::shared_ptr<Protocol> protocol_;

    //! serial_port name
    std::string serial_port_;
};
}
#endif //ROBORTS_SDK_HANDLE_H
