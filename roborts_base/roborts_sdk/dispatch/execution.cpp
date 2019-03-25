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

#include "execution.h"

namespace roborts_sdk
{
Executor::Executor(std::shared_ptr<Handle> handle) : handle_(handle) {}
std::shared_ptr<Handle> Executor::GetHandle()
{
    return handle_;
}

void Executor::ExecuteSubscription(std::shared_ptr<SubscriptionBase> subscription)
{
}
void Executor::ExecuteService(std::shared_ptr<ServiceBase> service)
{
}
void Executor::ExecuteClient(std::shared_ptr<ClientBase> client)
{
}
}
